package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SorterLogicColor {

    Telemetry telemetry;
    DcMotorEx sorterMotor;
    ColorSensor opticalSorterHoming;
    ColorSensor sorterColorSensor;


    // ---------- DASHBOARD-TUNABLE CONSTANTS ----------

    // Homing
    public static int HOME_THRESHOLD = 4250;      // tape detection alpha threshold
    public static double HOMING_VELOCITY = 150;   // homing speed (ticks/s)
    public static int HOMING_TIMEOUT_MS = 2500;   // each homing spin attempt
    public static int HOMING_REVERSE_TIMEOUT_MS = 2000;
    public static int HOMING_CREEP_TIMEOUT_MS = 1500;
    public static int HOMING_CREEP_VEL = 100;
    public static int HOMING_SETTLE_MS = 500;

    // Pocket positions (encoder ticks)
    public static int B1_INTAKE = 0;
    public static int B1_OUTTAKE = -259;

    public static int B2_INTAKE = 176;
    public static int B2_OUTTAKE = -80;

    // Match the tuned Manual class values
    public static int B3_INTAKE = 373;
    public static int B3_OUTTAKE = 114;

    // Positioning
    public static int POSITION_TOLERANCE = 4;     // acceptable final error in ticks
    public static double Kp = 0.8;                // kept for dashboard visibility (not really used now)
    public static double Kd = 0.35;
    public static int MAX_VEL = 1000;              // hard max velocity (ticks/s)

    // Zone thresholds (distance to target in ticks)
    public static int ZONE1_THRESH = 120;  // was 150
    public static int ZONE2_THRESH = 70;   // was 80
    public static int ZONE3_THRESH = 30;   // was 40
    public static int ZONE4_THRESH = 10;   // was 15

    // Zone velocities (ticks/s)
    public static int VEL_ZONE1 = 800;   // was 600
    public static int VEL_ZONE2 = 600;   // was 400
    public static int VEL_ZONE3 = 350;   // was 250
    public static int VEL_ZONE4 = 200;    // was 150
    public static int VEL_ZONE5 = 100;    // was 80

    // Smooth final approach controller
    public static double FINAL_K = 8.0;       // strength of final approach
    public static int FINAL_CLAMP = 200;      // max speed near target
    public static int REWIND_VEL = 350;

    // Telemetry throttling
    public static long TELEMETRY_INTERVAL_MS = 120;
    public static double UPDATE_PERIOD_MS = 10.0;

    // ---------- INTERNAL STATE (NOT CONFIG) ----------

    // Throttled telemetry
    private long lastTelem = 0;

    // States
    boolean sorterHomed = false;
    boolean homingFault = false;
    boolean tuned = false;
    boolean tapeSeen = false;

    int homingAttempts = 0;
    final int MAX_ATTEMPTS = 10;

    // Motion state
    int targetPos = 0;
    boolean moving = false;
    int lastError = 0;
    int stableCount = 0;

    public enum BallColor { PURPLE, GREEN, UNKNOWN }

    // Raw cached sensor values
    private int cachedAlpha = 0;
    private int cachedR = 0;
    private int cachedG = 0;
    private int cachedB = 0;

    public BallColor[] pocketColors = new BallColor[] {
            BallColor.UNKNOWN, // Pocket 1
            BallColor.UNKNOWN, // Pocket 2
            BallColor.UNKNOWN  // Pocket 3
    };

    public boolean[] pocketReady = {true, true, true};


    private int currentIntakePocket = 1; // starts at pocket 1

    private ElapsedTime updateRate = new ElapsedTime();

    // ================= TELEMETRY WRAPPER =================
    private void safeTelemetry(Runnable r) {
        long now = System.currentTimeMillis();
        if (now - lastTelem >= TELEMETRY_INTERVAL_MS) {
            r.run();
            lastTelem = now;
        }
    }

    // ================= INIT =================
    public void init(HardwareMap hwMap, Telemetry tel) {
        this.telemetry = tel;

        sorterMotor = hwMap.get(DcMotorEx.class, "sorterMotor");
        opticalSorterHoming = hwMap.get(ColorSensor.class, "opticalSorterHoming");
        sorterColorSensor = hwMap.get(ColorSensor.class, "sorterColorSensor");

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        // Start at pocket 1 intake
        targetPos = B1_INTAKE;
        moving = false;

        updateRate.reset();
    }

    // ================= HOMING (blocking, safe in init) =================
    public void homeSorter() {

        if (sorterHomed || homingFault) return;

        homingAttempts++;

        if (homingAttempts > MAX_ATTEMPTS) {
            homingFault = true;
            sorterMotor.setVelocity(0);
            return;
        }

        tapeSeen = false;
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        long attemptStart = System.currentTimeMillis();

        // Spin forward until we see tape or timeout
        while (!tapeSeen) {

            int alpha = opticalSorterHoming.alpha();
            sorterMotor.setVelocity(HOMING_VELOCITY);

            if (System.currentTimeMillis() - attemptStart > HOMING_TIMEOUT_MS) {
                sorterMotor.setVelocity(0);
                return;
            }

            if (alpha >= HOME_THRESHOLD) {
                tapeSeen = true;
                sorterMotor.setVelocity(0);
                try { Thread.sleep(HOMING_SETTLE_MS); } catch (Exception ignored) {}
                break;
            }

            try { Thread.sleep(5); } catch (Exception ignored) {}
        }

        int alpha = opticalSorterHoming.alpha();

        // If we lost tape, back up slowly until we see it again
        if (alpha < HOME_THRESHOLD) {

            long reverseStart = System.currentTimeMillis();
            sorterMotor.setVelocity(-HOMING_VELOCITY);

            while (opticalSorterHoming.alpha() < HOME_THRESHOLD) {
                if (System.currentTimeMillis() - reverseStart > HOMING_REVERSE_TIMEOUT_MS) {
                    sorterMotor.setVelocity(0);
                    return;
                }
            }

            sorterMotor.setVelocity(0);
            try { Thread.sleep(HOMING_SETTLE_MS); } catch (Exception ignored) {}
        }

        int a1 = opticalSorterHoming.alpha();
        try { Thread.sleep(30); } catch (Exception ignored) {}
        int a2 = opticalSorterHoming.alpha();

        // If still noisy, creep until firmly on tape
        if (a1 < HOME_THRESHOLD || a2 < HOME_THRESHOLD) {

            sorterMotor.setVelocity(HOMING_CREEP_VEL);

            long creepStart = System.currentTimeMillis();
            while (opticalSorterHoming.alpha() < HOME_THRESHOLD) {

                if (System.currentTimeMillis() - creepStart > HOMING_CREEP_TIMEOUT_MS) {
                    sorterMotor.setVelocity(0);
                    return;
                }
            }

            sorterMotor.setVelocity(0);
            try { Thread.sleep(HOMING_SETTLE_MS); } catch (Exception ignored) {}
        }

        if (opticalSorterHoming.alpha() < HOME_THRESHOLD) return;

        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterHomed = true;
        targetPos = B1_INTAKE;
        moving = false;

        currentIntakePocket = 1;
        pocketReady[0] = true;
    }

    // ================= COMMANDS =================
    public void goToPosition(int pos) {
        if (!sorterHomed || homingFault) return;
        targetPos = pos;
        moving = true;
        stableCount = 0;
    }

    public void goToIntake(int n) {
        currentIntakePocket = n;
        if (n == 1) goToPosition(B1_INTAKE);
        else if (n == 2) goToPosition(B2_INTAKE);
        else if (n == 3) goToPosition(B3_INTAKE);
    }

    public void goToOuttake(int n) {
        if (n == 1) goToPosition(B1_OUTTAKE);
        else if (n == 2) goToPosition(B2_OUTTAKE);
        else if (n == 3) goToPosition(B3_OUTTAKE);
    }

    public void markPocketReady(int pocket) {
        pocketReady[pocket - 1] = true;
    }

    // ================= PUBLIC ACCESSORS =================
    public boolean isAtIntakePosition() {
        int target;
        if (currentIntakePocket == 1) target = B1_INTAKE;
        else if (currentIntakePocket == 2) target = B2_INTAKE;
        else target = B3_INTAKE;

        return Math.abs(sorterMotor.getCurrentPosition() - target) <= POSITION_TOLERANCE;
    }

    public int getCurrentPos() {
        return sorterMotor.getCurrentPosition();
    }

    public int getTargetPos() {
        return targetPos;
    }

    public boolean isHomed() {
        return sorterHomed;
    }

    public boolean hasFault() {
        return homingFault;
    }

    public boolean isMoving() {
        return moving;
    }

    public SorterLogicColor.BallColor[] getPocketColors() {
        return pocketColors;
    }

    public void setCurrentIntakePocket(int n) {
        currentIntakePocket = n;
    }

    // ================= COLOR HELPERS =================
    private void updateColorCache() {
        cachedAlpha = sorterColorSensor.alpha();
        cachedR = sorterColorSensor.red();
        cachedG = sorterColorSensor.green();
        cachedB = sorterColorSensor.blue();
    }

    public BallColor detectBallColor() {
        // Update sensor values
        updateColorCache();

        // Ignore color when the sorter is moving
        if (moving) return BallColor.UNKNOWN;

        // Convert to float for stable comparisons
        float a = cachedAlpha / 1000f;
        float r = cachedR / 1000f;
        float g = cachedG / 1000f;
        float b = cachedB / 1000f;

        // 1. MUST be reading solid plastic, not a hole
        if (a < 0.20f) {
            return BallColor.UNKNOWN;
        }

        // 2. Reject ambiguous readings caused by holes
        float diffRG = Math.abs(r - g);
        float diffGB = Math.abs(g - b);
        float diffBR = Math.abs(b - r);

        if (diffRG < 0.001f && diffGB < 0.001f && diffBR < 0.001f) {
            return BallColor.UNKNOWN;
        }

        // 3. PURPLE detection — BLUE dominates
        if (b > g + 0.001f && b > r + 0.001f) {
            return BallColor.PURPLE;
        }

        // 4. GREEN detection — GREEN dominates
        if (g > b + 0.001f && g > r + 0.001f) {
            return BallColor.GREEN;
        }

        return BallColor.UNKNOWN;
    }

    public void storeColorForCurrentPocket(BallColor color) {

        if (color == BallColor.UNKNOWN) return;

        int idx = currentIntakePocket - 1;

        // Only overwrite when the pocket is ready for a new ball
        if (!pocketReady[idx]) return;

        pocketColors[idx] = color;

        // After storing a new ball, mark pocket as "occupied"
        pocketReady[idx] = false;
    }

    public Integer getPocketWithColor(BallColor color) {
        for (int i = 0; i < 3; i++) {
            if (pocketColors[i] == color) {
                return i + 1; // pockets are 1,2,3
            }
        }
        return null; // not found
    }

    public void clearPocket(int pocket) {
        int idx = pocket - 1;
        pocketColors[idx] = BallColor.UNKNOWN;
        pocketReady[idx] = true;
    }

    // ================= NON-BLOCKING UPDATE =================
    public void update() {
        // Keep color cache fresh every cycle
        updateColorCache();

        if (updateRate.milliseconds() < UPDATE_PERIOD_MS) return;
        updateRate.reset();

        if (homingFault) {
            sorterMotor.setVelocity(0);
            moving = false;
            return;
        }

        if (!moving) return;

        int current = sorterMotor.getCurrentPosition();
        int error = targetPos - current;
        int absError = Math.abs(error);

        safeTelemetry(() -> {
            telemetry.addData("Err", error);
            telemetry.addData("AbsErr", absError);
        });

        // ===================== REACHED TARGET =====================
        if (absError <= POSITION_TOLERANCE) {
            stableCount++;
            if (stableCount >= 3) {
                sorterMotor.setVelocity(0);
                moving = false;
                safeTelemetry(() -> telemetry.addData("CorrState", "HOLD_DONE"));
            } else {
                safeTelemetry(() -> telemetry.addData("CorrState", "HOLD_NUDGE"));
            }
            lastError = error;
            return;
        }

        stableCount = 0;

        // ===================== EXTREMELY CLOSE (<2) =====================
        if (absError <= 2) {
            sorterMotor.setVelocity(0);
            moving = false;
            safeTelemetry(() -> telemetry.addData("CorrState", "STOP_CLOSE"));
            lastError = error;
            return;
        }

        int vel;

        // ===================== FAR ZONES =====================
        if (absError > ZONE1_THRESH) {
            vel = VEL_ZONE1 * Integer.signum(error);
            safeTelemetry(() -> telemetry.addData("CorrState", "ZONE1"));
        }
        else if (absError > ZONE2_THRESH) {
            vel = VEL_ZONE2 * Integer.signum(error);
            safeTelemetry(() -> telemetry.addData("CorrState", "ZONE2"));
        }
        else if (absError > ZONE3_THRESH) {
            vel = VEL_ZONE3 * Integer.signum(error);
            safeTelemetry(() -> telemetry.addData("CorrState", "ZONE3"));
        }
        else if (absError > ZONE4_THRESH) {
            vel = VEL_ZONE4 * Integer.signum(error);
            safeTelemetry(() -> telemetry.addData("CorrState", "ZONE4"));
        }
        // ===================== FINAL APPROACH (SMOOTH MODE) =====================
        else {
            // velocity = k_final * error
            vel = (int)(FINAL_K * error);

            // clamp for smoothness
            if (vel > FINAL_CLAMP)  vel = FINAL_CLAMP;
            if (vel < -FINAL_CLAMP) vel = -FINAL_CLAMP;

            safeTelemetry(() -> telemetry.addData("CorrState", "FINAL_SMOOTH"));
        }

        // ===================== APPLY VELOCITY =====================
        if (vel > MAX_VEL) vel = MAX_VEL;
        if (vel < -MAX_VEL) vel = -MAX_VEL;

        sorterMotor.setVelocity(vel);
        lastError = error;
    }

    // ================= INIT LOOP =================
    public void init_loop() {

        if (homingFault) {
            safeTelemetry(() -> {
                telemetry.addLine("SORTER HOMING FAILED!");
                telemetry.addData("Attempts", homingAttempts);
                telemetry.update();
            });
            return;
        }

        if (!sorterHomed) {
            homeSorter();

            safeTelemetry(() -> {
                telemetry.addData("Homing", "IN PROGRESS");
                telemetry.addData("Attempts", homingAttempts);
                telemetry.addData("Alpha", opticalSorterHoming.alpha());
                telemetry.update();
            });
            return;
        }

        // Disable auto-tune; we just mark as tuned
        if (!tuned) {
            tuned = true;
        }

        update();

        safeTelemetry(() -> {
            telemetry.addData("Homing", "DONE");
            telemetry.addData("Tuned", tuned);
            telemetry.addData("Encoder", sorterMotor.getCurrentPosition());
            telemetry.addData("Kp", Kp);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        });
    }
}
