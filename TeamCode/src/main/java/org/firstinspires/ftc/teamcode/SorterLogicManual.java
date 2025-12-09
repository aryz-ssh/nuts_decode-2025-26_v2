package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class SorterLogicManual {

    Telemetry telemetry;
    DcMotorEx sorterMotor;
    ColorSensor opticalSorterHoming;

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

    public static int B3_INTAKE = 361;
    public static int B3_OUTTAKE = 110;

    // Positioning / PID
    public static int POSITION_TOLERANCE = 5;     // acceptable final error in ticks
    public static double Kp = 4.0;                // base proportional gain
    public static double Kd = 0.15;               // base derivative gain
    public static int MAX_VEL = 600;              // hard max velocity (ticks/s)

    // Zone thresholds (distance to target in ticks)
    public static int ZONE1_THRESH = 150;         // > this → full speed
    public static int ZONE2_THRESH = 80;          // 80–150
    public static int ZONE3_THRESH = 40;          // 40–80
    public static int ZONE4_THRESH = 15;          // 15–40

    // Zone velocities (ticks/s)
    public static int VEL_ZONE1 = 600;            // far / fast
    public static int VEL_ZONE2 = 400;
    public static int VEL_ZONE3 = 250;
    public static int VEL_ZONE4 = 150;
    public static int VEL_ZONE5 = 80;             // final creep

    // Overshoot correction (REWIND/SNAP)
    public static int REWIND_VEL = 600;
    public static int REWIND_TIME_MS = 80;
    public static int SNAP_VEL = 250;
    public static int SNAP_TIMEOUT_MS = 250;

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

    private enum CorrectionState { NONE, REWIND, SNAP }
    private CorrectionState correction = CorrectionState.NONE;

    private int rewindDirection = 0;
    private int snapDirection = 0;
    private long rewindStartTime = 0;
    private long snapStartTime = 0;

    // Motion state
    int targetPos = 0;
    boolean moving = false;
    int lastError = 0;
    int stableCount = 0;

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
    }

    // ================= AUTO TUNE =================
    public void autoTuneGains() {
        if (tuned || !sorterHomed || homingFault) return;

        sorterMotor.setVelocity(300);

        long startTime = System.currentTimeMillis();
        int startPos = sorterMotor.getCurrentPosition();

        while (System.currentTimeMillis() - startTime < 150) {
            // spin for 150ms
        }

        sorterMotor.setVelocity(0);

        int delta = Math.abs(sorterMotor.getCurrentPosition() - startPos);
        if (delta < 10) delta = 10;

        Kp = Math.max(2.0, Math.min(5.0, 350.0 / delta));
        Kd = Math.max(0.06, Math.min(0.25, delta / 400.0));

        tuned = true;
        sorterHomed = false;
        tapeSeen = false;
        homingAttempts = 0;
    }

    // ================= COMMANDS =================
    public void goToPosition(int pos) {
        if (!sorterHomed || homingFault) return;
        targetPos = pos;
        moving = true;
        stableCount = 0;
        correction = CorrectionState.NONE;
    }

    public void goToIntake(int n) {
        if (n == 1) goToPosition(B1_INTAKE);
        else if (n == 2) goToPosition(B2_INTAKE);
        else if (n == 3) goToPosition(B3_INTAKE);
    }

    public void goToOuttake(int n) {
        if (n == 1) goToPosition(B1_OUTTAKE);
        else if (n == 2) goToPosition(B2_OUTTAKE);
        else if (n == 3) goToPosition(B3_OUTTAKE);
    }

    // ================= PUBLIC ACCESSORS =================
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

    // ================= NON-BLOCKING UPDATE =================
    public void update() {

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
        long now = System.currentTimeMillis();

        safeTelemetry(() -> {
            telemetry.addData("Err", error);
            telemetry.addData("AbsErr", absError);
        });

        // ---------- Overshoot detection ----------
        boolean overshoot =
                (absError > POSITION_TOLERANCE) &&
                        (Math.abs(error) > Math.abs(lastError));

        if (correction == CorrectionState.NONE && overshoot) {
            correction = CorrectionState.REWIND;
            rewindDirection = (lastError > 0) ? -1 : 1;
            rewindStartTime = now;

            safeTelemetry(() -> telemetry.addData("CorrState", "START_REWIND"));
            lastError = error;
            return;
        }

        // ---------- REWIND ----------
        if (correction == CorrectionState.REWIND) {

            sorterMotor.setVelocity(REWIND_VEL * rewindDirection);

            if (now - rewindStartTime >= REWIND_TIME_MS) {
                sorterMotor.setVelocity(0);
                correction = CorrectionState.SNAP;
                snapStartTime = now;
                snapDirection = (targetPos - sorterMotor.getCurrentPosition() > 0) ? 1 : -1;
            }

            safeTelemetry(() -> telemetry.addData("CorrState", "REWIND"));
            lastError = error;
            return;
        }

        // ---------- SNAP BACK ----------
        if (correction == CorrectionState.SNAP) {

            sorterMotor.setVelocity(SNAP_VEL * snapDirection);

            boolean timeout = (now - snapStartTime >= SNAP_TIMEOUT_MS);

            if (absError <= POSITION_TOLERANCE) {
                sorterMotor.setVelocity(0);
                correction = CorrectionState.NONE;
                moving = false;
            } else if (timeout) {
                sorterMotor.setVelocity(0);
                correction = CorrectionState.NONE;
            }

            safeTelemetry(() -> telemetry.addData("CorrState", "SNAP"));
            lastError = error;

            if (correction != CorrectionState.NONE || !moving) return;
        }

        // ---------- MAIN MOTION CONTROL ----------
        if (absError > POSITION_TOLERANCE) {

            stableCount = 0;

            // If extremely close (<2 ticks), just stop to avoid chatter.
            if (absError <= 2) {
                sorterMotor.setVelocity(0);
                moving = false;
                lastError = error;
                return;
            }

            int vel;

            // 5-zone deceleration profile
            if (absError > ZONE1_THRESH) {
                vel = VEL_ZONE1 * Integer.signum(error);
            } else if (absError > ZONE2_THRESH) {
                vel = VEL_ZONE2 * Integer.signum(error);
            } else if (absError > ZONE3_THRESH) {
                vel = VEL_ZONE3 * Integer.signum(error);
            } else if (absError > ZONE4_THRESH) {
                vel = VEL_ZONE4 * Integer.signum(error);
            } else {
                vel = VEL_ZONE5 * Integer.signum(error);
            }

            // PD micro-correction (small adjustment)
            int pdBoost = (int)(Kp * 0.15 * error + Kd * 0.2 * (error - lastError));
            vel += pdBoost;

            // Clamp final velocity
            if (vel > MAX_VEL) vel = MAX_VEL;
            if (vel < -MAX_VEL) vel = -MAX_VEL;

            sorterMotor.setVelocity(vel);

            safeTelemetry(() -> telemetry.addData("CorrState", "CORRECTING"));

            lastError = error;
            return;
        }

        // ---------- Inside tolerance: wait to declare done ----------
        stableCount++;
        if (stableCount >= 3) {
            sorterMotor.setVelocity(0);
            moving = false;

            safeTelemetry(() -> telemetry.addData("CorrState", "HOLD_DONE"));
            lastError = error;
            return;
        }

        safeTelemetry(() -> telemetry.addData("CorrState", "HOLD_NUDGE"));
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

        if (!tuned) {
            autoTuneGains();

            safeTelemetry(() -> {
                telemetry.addData("Homing", "DONE");
                telemetry.addData("Tuning", "AUTO KD/KP...");
                telemetry.addData("Alpha", opticalSorterHoming.alpha());
                telemetry.update();
            });
            return;
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
