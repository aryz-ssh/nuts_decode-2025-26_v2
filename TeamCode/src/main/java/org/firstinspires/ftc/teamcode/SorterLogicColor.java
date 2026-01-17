package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

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

    // Overshoot recovery (snap-back)
    public static int RETURN_MIN_VEL = 220;    // speed used after crossing target
    public static int RETURN_MAX_VEL = 450;    // cap during snap-back
    public static int RETURN_ZONE_TICKS = 40;  // only do snap-back when near target
    public static int RETURN_HOLD_CYCLES = 2;  // cycles to keep snap-back active

    private int returnCountdown = 0;

    // ---------- DASHBOARD ----------
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private TelemetryPacket packet;

    // Dashboard signals
    public static double dbgTargetPos = 0;
    public static double dbgCurrentPos = 0;
    public static double dbgErrorGraph = 0;
    public static double dbgCmdVelocity = 0;
    public static double dbgActualVelocity = 0;

    // Pocket positions (encoder ticks)
    public static int B1_INTAKE = 0;
    public static int B1_OUTTAKE = -259;

    public static int B2_INTAKE = 176;
    public static int B2_OUTTAKE = -80;

    // Match the tuned Manual class values
    public static int B3_INTAKE = 373;
    public static int B3_OUTTAKE = 114;

    // Encoder wrap
    public static int TICKS_PER_REV = 537; // keep accurate (gobilda 5203 435rpm is 537.6 CPR at motor shaft)

    // Position tolerance / settle
    public static int POSITION_TOLERANCE = 3;     // acceptable final error in ticks
    public static int HOLD_STABLE_CYCLES = 3;     // how many consecutive cycles inside tolerance before stop

    // ======= NEW: SIMPLE POSITION→VELOCITY CONTROL =======
    // Think of this as: vel_cmd = K_POS * error
    // Higher = faster moves, but too high can overshoot / oscillate.
    public static double K_POS_FWD = 12.0;
    public static double K_POS_REV = 16.0; // stronger correction

    // Optional damping (usually you can keep this 0.0)
    // vel_cmd -= K_D * (error - lastError)
    public static double K_D = 0.0;               // start at 0; only add if you see bounce

    // Velocity limits
    public static int MAX_VEL = 1800;             // hard cap (ticks/s)
    public static int FINAL_CLAMP = 300;          // cap when close to target (prevents slamming)

    // “Stiction floor” to avoid getting stuck near target due to friction
    public static int MIN_VEL = 120;              // set 0 to disable

    // When do we switch to the close-range clamp?
    public static int CLAMP_ZONE_TICKS = 25;      // within this error, clamp to ±FINAL_CLAMP

    // Update pacing
    public static double UPDATE_PERIOD_MS = 5.0;

    // ---------- INTERNAL STATE (NOT CONFIG) ----------

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

    private boolean ballJustStored = false;

    public enum BallColor { PURPLE, GREEN, UNKNOWN }

    // ---------- OUTTAKE BEAM BREAK ----------
    private DigitalChannel outtakeBeamBreak;

    // Edge detect
    private boolean lastBeamBroken = false;
    private boolean ballExitEvent = false;     // one-shot event for TeleOp
    private long lastBeamBrokenMs = 0;

    // Which pocket are we currently trying to outtake?
    private int currentOuttakePocket = 0;
    private BallColor currentOuttakeColor = BallColor.UNKNOWN;

    // ---------- RGB STATUS LED (GoBILDA PWM Indicator) ----------
    private Servo statusLEDServo;
    private StatusLED_RGB statusLED;

    // These are *servo positions* (0..1) that your GoBILDA PWM LED interprets as colors.
    // You MUST tune these based on your LED (because the mapping can vary).
    public static double LED_OFF     = 0.00;
    public static double LED_RED     = 0.10;
    public static double LED_ORANGE  = 0.20;
    public static double LED_YELLOW  = 0.30;
    public static double LED_GREEN   = 0.50;
    public static double LED_BLUE    = 0.56;
    public static double LED_PURPLE  = 0.72;
    public static double LED_WHITE   = 1.00;

    // Blink timing
    public static int BLINK_PERIOD_MS_FAST = 150;   // for attention states (moving)
    public static int BLINK_PERIOD_MS_SLOW = 300;   // for outtake flashing
    public static int BLINK_PERIOD_MS_HOMING = 500;   // for homing
    private boolean shotConfirmed = false;   // latch RED
    private boolean shotAttempted = false;   // kicker pressed

    // Priority “flash red” when beam break confirms ball left
    public static int RED_FLASH_MS = 180;
    private long redFlashUntilMs = 0;

    private boolean outtakeWindowActive = false;
    private ElapsedTime outtakeWindowTimer = new ElapsedTime();
    public static double OUTTAKE_WINDOW_SEC = 0.75;

    // ---------- COLOR TUNABLES / STATE ----------
    private BallColor latchedColor = BallColor.UNKNOWN;
    private boolean colorLatched = false;

    // Raw cached sensor values
    private int cachedAlpha = 0;
    private int cachedR = 0;
    private int cachedG = 0;
    private int cachedB = 0;

    public BallColor[] pocketColors = new BallColor[] {
            BallColor.UNKNOWN,
            BallColor.UNKNOWN,
            BallColor.UNKNOWN
    };

    public boolean[] pocketReady = {true, true, true};
    private int currentIntakePocket = 1;

    private ElapsedTime updateRate = new ElapsedTime();

    // ---------- DEBUG STATE ----------
    private int dbgError = 0;
    private int dbgAbsError = 0;
    private String dbgCorrState = "IDLE";
    private boolean dbgHoming = false;
    public boolean autoAdvanceEnabled = false;

    // Debug signals (Dashboard graph-friendly if you choose to pipe them)
    public static double dbgVelCmd = 0;
    public static double dbgVelActual = 0;

    // ================= INIT =================
    public void init(HardwareMap hwMap, Telemetry tel) {
        this.telemetry = tel;

        sorterMotor = hwMap.get(DcMotorEx.class, "sorterMotor");
        opticalSorterHoming = hwMap.get(ColorSensor.class, "opticalSorterHoming");
        sorterColorSensor = hwMap.get(ColorSensor.class, "sorterColorSensor");
        outtakeBeamBreak = hwMap.get(DigitalChannel.class, "outtakeBeamBreak");
        statusLEDServo = hwMap.get(Servo.class, "statusLED");
        statusLED = new StatusLED_RGB(statusLEDServo);

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        outtakeBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        statusLED.setState(StatusLED_RGB.LEDState.WHITE);

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
        stableCount = 0;
        lastError = 0;
    }

    // ================= COMMANDS =================
    public void goToPosition(int pos) {
        if (!sorterHomed || homingFault) return;
        targetPos = pos;
        moving = true;
        stableCount = 0;
        lastError = 0;

        // reset color latch any time we command motion
        colorLatched = false;
        latchedColor = BallColor.UNKNOWN;
    }

    public void goToIntake(int n) {
        shotConfirmed = false;
        shotAttempted = false;

        currentOuttakePocket = 0;
        currentOuttakeColor = BallColor.UNKNOWN;

        currentIntakePocket = n;
        if (n == 1) goToPosition(B1_INTAKE);
        else if (n == 2) goToPosition(B2_INTAKE);
        else if (n == 3) goToPosition(B3_INTAKE);
    }

    public void goToOuttake(int n) {
        shotConfirmed = false;
        shotAttempted = false;

        currentOuttakePocket = n;
        currentOuttakeColor = pocketColors[n - 1];

        if (n == 1) goToPosition(B1_OUTTAKE);
        else if (n == 2) goToPosition(B2_OUTTAKE);
        else if (n == 3) goToPosition(B3_OUTTAKE);
    }

    // ================= PUBLIC ACCESSORS =================
    public boolean isAtIntakePosition() {
        int target;
        if (currentIntakePocket == 1) target = B1_INTAKE;
        else if (currentIntakePocket == 2) target = B2_INTAKE;
        else target = B3_INTAKE;

        return Math.abs(sorterMotor.getCurrentPosition() - target) <= POSITION_TOLERANCE;
    }

    public boolean isAtOuttakePosition(int pocket) {
        int target;
        if (pocket == 1) target = B1_OUTTAKE;
        else if (pocket == 2) target = B2_OUTTAKE;
        else target = B3_OUTTAKE;

        return Math.abs(sorterMotor.getCurrentPosition() - target) <= POSITION_TOLERANCE;
    }

    public int getCurrentPos() { return sorterMotor.getCurrentPosition(); }
    public int getTargetPos() { return targetPos; }
    public boolean isHomed() { return sorterHomed; }
    public boolean hasFault() { return homingFault; }
    public boolean isMoving() { return moving; }
    public BallColor[] getPocketColors() { return pocketColors; }
    public void setCurrentIntakePocket(int n) { currentIntakePocket = n; }
    public int getCurrentIntakePocket() { return currentIntakePocket; }
    private boolean readBeamBroken() { return outtakeBeamBreak.getState(); } // ACTIVE HIGH

    public void markPocketReady(int pocket) {
        pocketReady[pocket - 1] = true;
    }

    // ================= COLOR HELPERS =================
    private void updateColorCache() {
        cachedAlpha = sorterColorSensor.alpha();
        cachedR = sorterColorSensor.red();
        cachedG = sorterColorSensor.green();
        cachedB = sorterColorSensor.blue();
    }

    public BallColor detectBallColor() {
        updateColorCache();

        if (colorLatched) return latchedColor;
        if (!isAtIntakePosition()) return BallColor.UNKNOWN;

        float a = cachedAlpha / 1000f;
        float r = cachedR / 1000f;
        float g = cachedG / 1000f;
        float b = cachedB / 1000f;

        if (a < 0.15f) return BallColor.UNKNOWN;

        float margin = 0.001f;

        if (b > g + margin && b > r + margin) {
            latchedColor = BallColor.PURPLE;
            colorLatched = true;
            return latchedColor;
        }

        if (g > b + margin && g > r + margin) {
            latchedColor = BallColor.GREEN;
            colorLatched = true;
            return latchedColor;
        }

        return BallColor.UNKNOWN;
    }

    public void storeColorForCurrentPocket(BallColor color) {
        if (color == BallColor.UNKNOWN) return;

        int idx = currentIntakePocket - 1;
        if (!pocketReady[idx]) return;

        pocketColors[idx] = color;
        pocketReady[idx] = false;

        ballJustStored = true;

        // clear latch after storing
        colorLatched = false;
        latchedColor = BallColor.UNKNOWN;
    }

    public void onBallExitedOuttake() {

        if (currentOuttakePocket >= 1 && currentOuttakePocket <= 3) {
            clearPocket(currentOuttakePocket);
        }

        shotConfirmed = true;
        shotAttempted = false;
        outtakeWindowActive = false;

        currentOuttakePocket = 0;
        currentOuttakeColor = BallColor.UNKNOWN;
    }

    public boolean consumeBallJustStored() {
        if (ballJustStored) {
            ballJustStored = false;
            return true;
        }
        return false;
    }

    public Integer getPocketWithColor(BallColor color) {
        for (int i = 0; i < 3; i++) {
            if (pocketColors[i] == color) return i + 1;
        }
        return null;
    }

    public void clearPocket(int pocket) {
        int idx = pocket - 1;
        pocketColors[idx] = BallColor.UNKNOWN;
        pocketReady[idx] = true;
    }

    public void beginOuttakeVerification(int pocket) {
        currentOuttakePocket = pocket;
        currentOuttakeColor = pocketColors[pocket - 1];

        shotAttempted = true;
        shotConfirmed = false;

        lastBeamBroken = readBeamBroken();
        outtakeWindowActive = true;
        outtakeWindowTimer.reset();
    }

    private void setRuntimeLED() {
        // 1) Shot confirmed → RED (latched)
        if (shotConfirmed) {
            statusLED.setState(StatusLED_RGB.LEDState.RED);
            return;
        }

        // 2) Shot attempt (kicker active)
        if (shotAttempted) {
            statusLED.setState(StatusLED_RGB.LEDState.YELLOW);
            return;
        }

        // 3) Show selected pocket color
        BallColor color = BallColor.UNKNOWN;

        if (currentOuttakePocket != 0) {
            color = pocketColors[currentOuttakePocket - 1];
        } else if (currentIntakePocket != 0) {
            color = pocketColors[currentIntakePocket - 1];
        }

        if (color == BallColor.GREEN) {
            statusLED.setState(StatusLED_RGB.LEDState.GREEN);
        }
        else if (color == BallColor.PURPLE) {
            statusLED.setState(StatusLED_RGB.LEDState.PURPLE);
        }
        else {
            statusLED.setState(StatusLED_RGB.LEDState.WHITE);
        }
    }

    public void assumePreloadColors() {
        pocketColors[0] = BallColor.GREEN;   // Pocket 1
        pocketColors[1] = BallColor.PURPLE;  // Pocket 2
        pocketColors[2] = BallColor.PURPLE;  // Pocket 3

        pocketReady[0] = false;
        pocketReady[1] = false;
        pocketReady[2] = false;
    }

    // ================= NON-BLOCKING UPDATE =================
    public void update() {

        if (updateRate.milliseconds() < UPDATE_PERIOD_MS) return;
        updateRate.reset();

        long now = System.currentTimeMillis();

        // --- 1) Beam break window ALWAYS runs ---
        boolean beamBroken = readBeamBroken(); // ACTIVE HIGH per your statement

        if (outtakeWindowActive) {
            if (lastBeamBroken && !beamBroken) {
                onBallExitedOuttake();
            }
            else if (outtakeWindowTimer.seconds() > OUTTAKE_WINDOW_SEC) {
                // Shot failed → revert
                outtakeWindowActive = false;
                shotAttempted = false;   // <-- revert to pocket color
            }
        }
        lastBeamBroken = beamBroken;

        // --- 2) LEDs (runtime) can update even if not moving ---
        setRuntimeLED();

        if (homingFault) {
            sorterMotor.setVelocity(0);
            moving = false;
            dbgCorrState = "FAULT";
            return;
        }

        if (!moving) {
            dbgCorrState = "IDLE";
            return;
        }

        int current = sorterMotor.getCurrentPosition();
        int error = targetPos - current;
        int absError = Math.abs(error);

        // detect crossing
        boolean crossedTarget =
                (error != 0) &&
                        (lastError != 0) &&
                        ((error > 0) != (lastError > 0));

        if (crossedTarget && absError <= RETURN_ZONE_TICKS) {
            returnCountdown = RETURN_HOLD_CYCLES;
        }

        // reached target
        if (absError <= POSITION_TOLERANCE) {
            stableCount++;
            if (stableCount >= HOLD_STABLE_CYCLES) {
                sorterMotor.setVelocity(0);
                moving = false;
                dbgCorrState = "HOLD_DONE";
            } else {
                dbgCorrState = "HOLD_NUDGE";
            }
            lastError = error;
            return;
        }

        stableCount = 0;

        double velCmdD;

        // ================= SNAP-BACK MODE =================
        if (returnCountdown > 0) {
            returnCountdown--;

            velCmdD = Math.signum(error)
                    * Math.max(RETURN_MIN_VEL, absError * K_POS_REV);

            velCmdD = Math.min(Math.abs(velCmdD), RETURN_MAX_VEL)
                    * Math.signum(velCmdD);

            dbgCorrState = "RETURN";
        }
        // ================= NORMAL MODE =================
        else {
            double k = (error > 0) ? K_POS_FWD : K_POS_REV;
            velCmdD = k * error;

            if (absError <= CLAMP_ZONE_TICKS) {
                velCmdD = Math.max(
                        Math.min(velCmdD, FINAL_CLAMP),
                        -FINAL_CLAMP
                );
                dbgCorrState = "CLAMP";
            } else {
                dbgCorrState = "CRUISE";
            }
        }

        // global clamp
        velCmdD = Math.max(
                Math.min(velCmdD, MAX_VEL),
                -MAX_VEL
        );

        // stiction floor (only outside clamp)
        if (absError > CLAMP_ZONE_TICKS &&
                Math.abs(velCmdD) < MIN_VEL &&
                Math.abs(velCmdD) > 0) {
            velCmdD = MIN_VEL * Math.signum(velCmdD);
        }

        int velCmd = (int) velCmdD;
        sorterMotor.setVelocity(velCmd);

        dbgVelCmd = velCmd;
        dbgVelActual = sorterMotor.getVelocity();

        dbgTargetPos = targetPos;
        dbgCurrentPos = current;
        dbgErrorGraph = error;
        dbgCmdVelocity = velCmd;
        dbgActualVelocity = sorterMotor.getVelocity();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("error", error);
        packet.put("cmdVelocity", velCmd);
        packet.put("state", dbgCorrState);
        dashboard.sendTelemetryPacket(packet);

        lastError = error;
    }

    // ================= INIT LOOP =================
    public void init_loop() {

        long now = System.currentTimeMillis();

        // HARD FAULT
        if (homingFault) {
            statusLED.setState(StatusLED_RGB.LEDState.RED);
            return;
        }

        // NOT HOMED YET → FLASH ORANGE
        if (!sorterHomed) {
            statusLED.setState(StatusLED_RGB.LEDState.ORANGE);
            homeSorter();
            dbgHoming = true;
            return;
        }

        // HOMED → SOLID BLUE
        statusLED.setState(StatusLED_RGB.LEDState.CYAN);
        if (!tuned) tuned = true;

        dbgHoming = false;
    }

    // ================= DEBUG GETTERS =================
    public int getDebugError() { return dbgError; }
    public int getDebugAbsError() { return dbgAbsError; }
    public String getDebugCorrState() { return dbgCorrState; }
    public boolean isDebugHoming() { return dbgHoming; }
    public int getHomingAttempts() { return homingAttempts; }
}
