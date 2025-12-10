/* package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class SorterLogicV2_indexing_experiment {

    Telemetry telemetry;
    DcMotorEx sorterMotor;
    ColorSensor opticalSorterHoming;
    ColorSensor sorterColorSensor;

    // Throttled telemetry
    private long lastTelem = 0;
    private static final long TELEMETRY_INTERVAL = 120; // ms

    // Cached sensor values
    private int cachedAlpha = 0;
    private int cachedR = 0;
    private int cachedG = 0;
    private int cachedB = 0;
    private long lastSensorRead = 0;

    // States
    public boolean sorterHomed = false;
    boolean homingFault = false;
    public boolean tuned = false;
    boolean tapeSeen = false;

    int homingAttempts = 0;
    final int MAX_ATTEMPTS = 10;

    private enum CorrectionState { NONE, REWIND, SNAP }
    private CorrectionState correction = CorrectionState.NONE;

    private int rewindDirection = 0;
    private int snapDirection = 0;
    private long rewindStartTime = 0;
    private long snapStartTime = 0;

    int HOME_THRESHOLD = 4250;
    double HOMING_VELOCITY = 150;

    // Pocket positions
    public int B1_INTAKE = 0;
    public int B1_OUTTAKE = -259;

    public int B2_INTAKE = 176;
    public int B2_OUTTAKE = -80;

    public int B3_INTAKE = 361;
    public int B3_OUTTAKE = 110;

    int targetPos = 0;
    boolean moving = false;
    int POSITION_TOLERANCE = 5;

    double Kp = 4.0;
    double Kd = 0.15;

    int MAX_VEL = 600;
    int SLOW_ZONE = 90;

    int lastError = 0;
    int stableCount = 0;



    private ElapsedTime updateRate = new ElapsedTime();
    private static final double UPDATE_PERIOD_MS = 10.0;

    public enum BallColor { PURPLE, GREEN, UNKNOWN }
    private int presenceCount = 0;
    private static final int PRESENCE_FRAMES = 3;
    private BallColor lastRawColor = BallColor.UNKNOWN;
    private BallColor stableColor = BallColor.UNKNOWN;
    private int colorStableCount = 0;
    private static final int COLOR_FRAMES = 3;
    private static final int COLOR_ALPHA_MIN = 260;

    // ===== Auto-intake / indexing logic =====
    private enum IntakeState {
        IDLE,          // Doing nothing
        WAIT_FOR_BALL, // Parked at intake pocket waiting for ball
        WOBBLE_COLOR,  // Rotate slightly until color detected
        ADVANCE,       // Move to next pocket
        FINISHED       // All pockets filled → go to outtake
    }

    private boolean advanceInProgress = false;

    private IntakeState intakeState = IntakeState.IDLE;
    private int intakePocketIndex = 1; // 1 → B1, 2 → B2, 3 → B3
    private int ballsCollected = 0;
    private BallColor[] pocketAssignment = new BallColor[4]; // ignore index 0
    private long wobbleStartTime = 0;
    private long pocketArriveTime = 0;


    // =============== THROTTLED TELEMETRY WRAPPER ===============
    private void safeTelemetry(Runnable r) {
        long now = System.currentTimeMillis();
        if (now - lastTelem >= TELEMETRY_INTERVAL) {
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

        updateRate.reset();
    }

    // ================= SENSOR CACHE =================
    public void updateSensors() {
        if (System.currentTimeMillis() - lastSensorRead > 20) {
            cachedAlpha = sorterColorSensor.alpha();
            cachedR = sorterColorSensor.red();
            cachedG = sorterColorSensor.green();
            cachedB = sorterColorSensor.blue();
            lastSensorRead = System.currentTimeMillis();
        }
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

        while (!tapeSeen) {

            int alpha = opticalSorterHoming.alpha();
            sorterMotor.setVelocity(HOMING_VELOCITY);

            if (System.currentTimeMillis() - attemptStart > 2500) {
                sorterMotor.setVelocity(0);
                return;
            }

            if (alpha >= HOME_THRESHOLD) {
                tapeSeen = true;
                sorterMotor.setVelocity(0);
                try { Thread.sleep(500); } catch (Exception ignored) {}
                break;
            }

            try { Thread.sleep(5); } catch (Exception ignored) {}
        }

        int alpha = opticalSorterHoming.alpha();

        if (alpha < HOME_THRESHOLD) {

            long reverseStart = System.currentTimeMillis();
            sorterMotor.setVelocity(-HOMING_VELOCITY);

            while (opticalSorterHoming.alpha() < HOME_THRESHOLD) {
                if (System.currentTimeMillis() - reverseStart > 2000) {
                    sorterMotor.setVelocity(0);
                    return;
                }
            }

            sorterMotor.setVelocity(0);
            try { Thread.sleep(500); } catch (Exception ignored) {}
        }

        int a1 = opticalSorterHoming.alpha();
        try { Thread.sleep(30); } catch (Exception ignored) {}
        int a2 = opticalSorterHoming.alpha();

        if (a1 < HOME_THRESHOLD || a2 < HOME_THRESHOLD) {

            sorterMotor.setVelocity(100);

            long creepStart = System.currentTimeMillis();
            while (opticalSorterHoming.alpha() < HOME_THRESHOLD) {

                if (System.currentTimeMillis() - creepStart > 1500) {
                    sorterMotor.setVelocity(0);
                    return;
                }
            }

            sorterMotor.setVelocity(0);
            try { Thread.sleep(500); } catch (Exception ignored) {}
        }

        if (opticalSorterHoming.alpha() < HOME_THRESHOLD) return;

        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterHomed = true;
    }

    // ================= AUTO TUNE =================
    public void autoTuneGains() {
        if (tuned || !sorterHomed || homingFault) return;

        sorterMotor.setVelocity(300);

        long startTime = System.currentTimeMillis();
        int startPos = sorterMotor.getCurrentPosition();

        while (System.currentTimeMillis() - startTime < 150) {}

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
        if (n == 2) goToPosition(B2_INTAKE);
        if (n == 3) goToPosition(B3_INTAKE);
    }

    public void goToOuttake(int n) {
        if (n == 1) goToPosition(B1_OUTTAKE);
        if (n == 2) goToPosition(B2_OUTTAKE);
        if (n == 3) goToPosition(B3_OUTTAKE);
    }

    public boolean isBallPresent() {
        int a = cachedAlpha;
        int maxRGB = Math.max(cachedR, Math.max(cachedG, cachedB));

        // EMPTY
        if (a < 120 && maxRGB < 150) {
            if (presenceCount > 0) presenceCount--;
            return false;
        }

        // NOT EMPTY → ball OR hole
        if (presenceCount < PRESENCE_FRAMES) presenceCount++;
        return presenceCount >= PRESENCE_FRAMES;
    }

    private BallColor detectBallColorRaw() {
        int r = cachedR;
        int g = cachedG;
        int b = cachedB;
        int a = cachedAlpha;

        // must be looking at solid plastic
        if (a < COLOR_ALPHA_MIN) {
            return BallColor.UNKNOWN;
        }

        // GREEN — your readings:
        // G = 521, R = 140, B = 381
        if (g >= 250 &&
                g > r + 80 &&
                g > b + 60) {
            return BallColor.GREEN;
        }

        // PURPLE — your readings:
        // B = 559, G = 370, R = 276
        if (b >= 350 &&
                b > g + 80 &&
                b > r + 80) {
            return BallColor.PURPLE;
        }

        return BallColor.UNKNOWN;
    }

    public BallColor detectBallColor() {
        BallColor raw = detectBallColorRaw();

        if (raw != BallColor.UNKNOWN && raw == lastRawColor) {
            if (colorStableCount < COLOR_FRAMES) colorStableCount++;
        } else {
            colorStableCount = 0;
        }

        lastRawColor = raw;

        if (colorStableCount >= COLOR_FRAMES) {
            stableColor = raw;
        }

        return stableColor;
    }

    public int getAlpha() { return cachedAlpha; }
    public int getRed()   { return cachedR; }
    public int getGreen() { return cachedG; }
    public int getBlue()  { return cachedB; }

    public void startAutoIntake() {

        if (!sorterHomed || homingFault) return;

        intakeState = IntakeState.WAIT_FOR_BALL;

        intakePocketIndex = 1;
        ballsCollected = 0;
        advanceInProgress = false;

        // Reset stored pocket colors
        for (int i = 1; i <= 3; i++) {
            pocketAssignment[i] = BallColor.UNKNOWN;
        }

        pocketArriveTime = System.currentTimeMillis();

        goToIntake(intakePocketIndex);
    }

    private void wobbleForColor() {
        long now = System.currentTimeMillis();

        // If we see color → done
        BallColor c = detectBallColor();
        if (c != BallColor.UNKNOWN) {
            // store result
            pocketAssignment[intakePocketIndex] = c;
            ballsCollected++;

            // stop wobbling
            sorterMotor.setVelocity(0);

            // go advance
            intakeState = IntakeState.ADVANCE;
            return;
        }

        // If we've been wobbling too long and still no solid plastic detected
        if (now - wobbleStartTime > 600) {

            // Stop wobbling
            sorterMotor.setVelocity(0);

            // Go back to waiting — because this wasn't a valid ball
            intakeState = IntakeState.WAIT_FOR_BALL;

            // Wait again for a ball to actually appear
            return;
        }

        // Otherwise wobble slowly
        sorterMotor.setVelocity(80); // gentle wobble
    }

    public int getCurrentPos() { return sorterMotor.getCurrentPosition(); }
    public int getTargetPos() { return targetPos; }

    // ================= NON-BLOCKING UPDATE =================
    public void update() {

        updateSensors();

        if (updateRate.milliseconds() < UPDATE_PERIOD_MS) return;
        updateRate.reset();

        if (homingFault) {
            sorterMotor.setVelocity(0);
            moving = false;
            return;
        }

        // ===== AUTO-INTAKE STATE MACHINE (ALWAYS RUNS) =====
        switch (intakeState) {

            case IDLE:
                break;

            case WAIT_FOR_BALL:

                // Allow PD settle time
                if (System.currentTimeMillis() - pocketArriveTime < 250)
                    return;

                // EMPTY pocket detection
                boolean emptyPocket = (cachedAlpha < 70 &&
                        cachedR < 90 &&
                        cachedG < 90 &&
                        cachedB < 90);

                if (!emptyPocket) {
                    // Something is there: ball, hole, edge of ball, etc.
                    wobbleStartTime = System.currentTimeMillis();
                    intakeState = IntakeState.WOBBLE_COLOR;
                }

                break;


            case WOBBLE_COLOR:
                // Wobble handles the motor itself and pauses PD.
                wobbleForColor();
                return;  // PD must not run while wobbling


            case ADVANCE:

                // Run ONCE per ball
                if (!advanceInProgress) {

                    advanceInProgress = true;

                    if (ballsCollected >= 3) {
                        // All pockets done -> send pocket 1 to outtake
                        goToOuttake(1);
                        intakeState = IntakeState.FINISHED;
                    } else {
                        // Move to next pocket
                        intakePocketIndex++;
                        if (intakePocketIndex > 3) intakePocketIndex = 3;

                        goToIntake(intakePocketIndex);

                        // timestamp used by WAIT_FOR_BALL settle timer
                        pocketArriveTime = System.currentTimeMillis();

                        intakeState = IntakeState.WAIT_FOR_BALL;
                    }

                    advanceInProgress = false;
                    break;  // CRITICAL: stop processing
                }

                // otherwise do nothing until PD finishes moving
                break;


            case FINISHED:
                // All done, PD holds final position
                break;
        }

        // ===== IF NOT MOVING TOWARD A POSITION, NOTHING MORE TO DO =====
        if (!moving) {
            return;
        }

        // ===== FROM HERE ON: NORMAL POSITION PD CONTROL =====

        int current = sorterMotor.getCurrentPosition();
        int error = targetPos - current;
        int absError = Math.abs(error);
        long now = System.currentTimeMillis();

        safeTelemetry(() -> {
            telemetry.addData("Err", error);
            telemetry.addData("AbsErr", absError);
        });

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

        if (correction == CorrectionState.REWIND) {

            sorterMotor.setVelocity(600 * rewindDirection);

            if (now - rewindStartTime >= 80) {
                sorterMotor.setVelocity(0);
                correction = CorrectionState.SNAP;
                snapStartTime = now;
                snapDirection = (targetPos - sorterMotor.getCurrentPosition() > 0) ? 1 : -1;
            }

            safeTelemetry(() -> telemetry.addData("CorrState", "REWIND"));
            lastError = error;
            return;
        }

        if (correction == CorrectionState.SNAP) {

            sorterMotor.setVelocity(250 * snapDirection);

            boolean timeout = (now - snapStartTime >= 250);

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

        if (absError > POSITION_TOLERANCE) {

            stableCount = 0;

            int vel = (int)(Kp * error + Kd * (error - lastError));

            vel = Math.max(-MAX_VEL, Math.min(MAX_VEL, vel));

            if (absError < SLOW_ZONE && Math.abs(vel) > 250)
                vel = 250 * Integer.signum(error);

            if (Math.abs(vel) < 80)
                vel = 80 * Integer.signum(error);

            sorterMotor.setVelocity(vel);

            safeTelemetry(() -> telemetry.addData("CorrState", "CORRECTING"));
            lastError = error;
            return;
        }

        stableCount++;
        if (stableCount >= 3) {
            sorterMotor.setVelocity(0);
            moving = false;
            advanceInProgress = false;
            pocketArriveTime = System.currentTimeMillis();

            safeTelemetry(() -> telemetry.addData("CorrState", "HOLD_DONE"));
            lastError = error;
            return;
        }

        int fineVel = (int)(error * (Kp * 0.25));
        if (Math.abs(fineVel) < 60)
            fineVel = 60 * Integer.signum(error);

        sorterMotor.setVelocity(fineVel);


        safeTelemetry(() -> telemetry.addData("CorrState", "HOLD_NUDGE"));
        lastError = error;
    }

    // ================= INIT LOOP =================
    public void init_loop() {

        updateSensors();

        if (homingFault) {
            safeTelemetry(() -> {
                telemetry.addLine("SORTER HOMING FAILED!");
                telemetry.addData("Attempts", homingAttempts);
                telemetry.addData("Alpha", cachedAlpha);
                telemetry.update();
            });
            return;
        }

        if (!sorterHomed) {
            homeSorter();

            safeTelemetry(() -> {
                telemetry.addData("Homing", "IN PROGRESS");
                telemetry.addData("Attempts", homingAttempts);
                telemetry.addData("Alpha", cachedAlpha);
                telemetry.update();
            });
            return;
        }

        if (!tuned) {
            autoTuneGains();

            safeTelemetry(() -> {
                telemetry.addData("Homing", "DONE");
                telemetry.addData("Tuning", "AUTO KD/KP...");
                telemetry.addData("Alpha", cachedAlpha);
                telemetry.update();
            });
            return;
        }

        update();

        safeTelemetry(() -> {
            telemetry.addData("Homing", "DONE");
            telemetry.addData("Tuned", tuned);
            telemetry.addData("Alpha", cachedAlpha);
            telemetry.addData("Encoder", sorterMotor.getCurrentPosition());
            telemetry.addData("Kp", Kp);
            telemetry.addData("Kd", Kd);
            telemetry.update();
        });
    }
}
*/