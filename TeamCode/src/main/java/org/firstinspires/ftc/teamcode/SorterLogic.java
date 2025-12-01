package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/*
    TREAT THIS AS ITS OWN CLASS TO BE INITIALIZED IN "Mechanisms.java"!
    SORTING LOGIC WILL GET COMPLEX.
*/

public class SorterLogic {
    Telemetry telemetry;
    DcMotorEx sorterMotor;
    ColorSensor opticalSorterHoming;   // HOMING ONLY
    ColorSensor sorterColorSensor;     // BALL COLOR ONLY

    // ---------------- STATES ----------------
    boolean sorterHomed = false;
    boolean homingFault = false;
    boolean tuned = false;
    boolean tapeSeen = false;

    int homingAttempts = 0;
    final int MAX_ATTEMPTS = 10;

    // Overshoot correction state machine
    private enum CorrectionState {
        NONE,
        REWIND,
        SNAP
    }

    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    private CorrectionState correction = CorrectionState.NONE;
    private int rewindDirection = 0;
    private int snapDirection = 0;
    private long rewindStartTime = 0;
    private long snapStartTime = 0;

    // Threshold for detecting reflective tape
    int HOME_THRESHOLD = 4250;

    // Homing
    double HOMING_VELOCITY = 150;

    // --- Pocket positions (in encoder ticks) ---
    public int B1_INTAKE  = 0;
    public int B1_OUTTAKE = -259;

    public int B2_INTAKE  = 176;
    public int B2_OUTTAKE = -80;

    public int B3_INTAKE  = 361;
    public int B3_OUTTAKE = 110;

    // --- Motion control state ---
    int targetPos = 0;          // Encoder ticks target
    boolean moving = false;     // Whether carousel is rotating
    int POSITION_TOLERANCE = 5; // acceptable error in ticks

    // --- PD control + auto tuning ---
    double Kp = 4.0;            // auto-tuned
    double Kd = 0.15;           // auto-tuned

    // Speed shaping (for slowing things down a bit)
    int MAX_VEL = 600;          // clamp for max PD velocity
    int SLOW_ZONE = 90;         // zone where we slow down more

    int lastError = 0;
    int stableCount = 0;        // require multiple stable cycles before stopping

    int BALL_PRESENT_THRESHOLD = 200;       // Ball presence threshold (tuned based on sensor values)


    public void init(HardwareMap hwMap, Telemetry tel) {
        this.telemetry = tel;

        sorterMotor = hwMap.get(DcMotorEx.class, "sorterMotor");

        opticalSorterHoming = hwMap.get(ColorSensor.class, "opticalSorterHoming");
        sorterColorSensor   = hwMap.get(ColorSensor.class, "sorterColorSensor");

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
    }

    // -------- BULLETPROOF HOMING --------
    public void homeSorter() {

        if (sorterHomed || homingFault) return;

        homingAttempts++;   // count attempt

        // If exceeded attempts → fail permanently
        if (homingAttempts > MAX_ATTEMPTS) {
            homingFault = true;
            sorterMotor.setVelocity(0);
            return;
        }

        tapeSeen = false;

        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        long attemptStart = System.currentTimeMillis();

        // STEP 1: Move forward until tape FIRST detected
        while (!tapeSeen) {

            int alpha = opticalSorterHoming.alpha();
            sorterMotor.setVelocity(HOMING_VELOCITY);

            // Safety timeout in case sensor fails entirely
            if (System.currentTimeMillis() - attemptStart > 2500) {
                sorterMotor.setVelocity(0);
                return; // let init_loop retry another attempt
            }

            if (alpha >= HOME_THRESHOLD) {
                tapeSeen = true;
                sorterMotor.setVelocity(0);
                try { Thread.sleep(500); } catch (Exception ignored) {}
                break;
            }
        }

        // STEP 2: Check if overshot OFF tape
        int alpha = opticalSorterHoming.alpha();

        if (alpha < HOME_THRESHOLD) {

            long reverseStart = System.currentTimeMillis();
            sorterMotor.setVelocity(-HOMING_VELOCITY);

            // reverse until ON tape OR timeout
            while (opticalSorterHoming.alpha() < HOME_THRESHOLD) {

                if (System.currentTimeMillis() - reverseStart > 2000) {
                    sorterMotor.setVelocity(0);
                    return;  // restart attempt
                }
            }

            sorterMotor.setVelocity(0);
            try { Thread.sleep(500); } catch (Exception ignored) {}
        }

        // STEP 3: Final check stability
        int a1 = opticalSorterHoming.alpha();
        try { Thread.sleep(30); } catch (Exception ignored) {}
        int a2 = opticalSorterHoming.alpha();

        if (a1 < HOME_THRESHOLD || a2 < HOME_THRESHOLD) {

            // creep back forward to tape
            sorterMotor.setVelocity(100);

            long creepStart = System.currentTimeMillis();
            while (opticalSorterHoming.alpha() < HOME_THRESHOLD) {

                if (System.currentTimeMillis() - creepStart > 1500) {
                    sorterMotor.setVelocity(0);
                    return; // redo attempt
                }
            }

            sorterMotor.setVelocity(0);
            try { Thread.sleep(500); } catch (Exception ignored) {}
        }

        // VALIDATION
        if (opticalSorterHoming.alpha() < HOME_THRESHOLD) {
            return;  // redo attempt
        }

        // SUCCESS
        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        sorterHomed = true;
    }

    // -------- AUTO-TUNE GAINS (Kp + Kd) --------
    public void autoTuneGains() {
        if (tuned || !sorterHomed || homingFault) return;

        sorterMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        sorterMotor.setVelocity(300); // gentle test speed (slightly slower)

        long startTime = System.currentTimeMillis();
        int startPos = sorterMotor.getCurrentPosition();

        // Small test move (~150ms)
        while (System.currentTimeMillis() - startTime < 150) {
            // let the motor run
        }

        sorterMotor.setVelocity(0);

        int endPos = sorterMotor.getCurrentPosition();
        int delta = Math.abs(endPos - startPos);
        if (delta < 10) delta = 10; // safety floor

        // --- Auto Kp --- (softened)
        double newKp = 350.0 / delta;     // was 450.0 / delta
        if (newKp < 2.0) newKp = 2.0;
        if (newKp > 5.0) newKp = 5.0;     // was capped at 7.0
        Kp = newKp;

        // --- Auto Kd --- (softened)
        double newKd = delta / 400.0;     // was delta / 300.0
        if (newKd < 0.06) newKd = 0.06;
        if (newKd > 0.25) newKd = 0.25;
        Kd = newKd;

        tuned = true;

        telemetry.addData("Auto-Kp", Kp);
        telemetry.addData("Auto-Kd", Kd);
        telemetry.addData("TuneDelta", delta);

        // Tuning done — now force a second homing cycle to re-align encoder to tape
        sorterHomed = false;
        tapeSeen = false;
        homingAttempts = 0;
    }

    public boolean isHomed() {
        return sorterHomed;
    }

    // -------- COMMANDS --------
    public void goToPosition(int pos) {
        if (!sorterHomed || homingFault) return;
        targetPos = pos;
        moving = true;
        stableCount = 0;
        correction = CorrectionState.NONE;   // reset correction state on new move
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
        int a = sorterColorSensor.alpha();
        return a > BALL_PRESENT_THRESHOLD;
    }

    public BallColor detectBallColor() {
        int r = sorterColorSensor.red();
        int g = sorterColorSensor.green();
        int b = sorterColorSensor.blue();

        // ---- PURPLE ----
        if (b > g + 40 &&
                b > r + 60 &&
                b > 350) {
            return BallColor.PURPLE;
        }

        // ---- GREEN ----
        if (g > r + 25 &&
                g > b + 25 &&
                g > 170 &&
                b < 400) {
            return BallColor.GREEN;
        }

        return BallColor.UNKNOWN;
    }

    // -------- MAIN CONTROL LOOP --------
    // Call this every OpMode loop
    public void update() {
        // ========================
        //   SAFETY BLOCK
        // ========================
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

        telemetry.addData("Err", error);
        telemetry.addData("AbsErr", absError);


        // ==========================================
        //   OVERSHOOT DETECTION  (START CORRECTION)
        // ==========================================
        if (correction == CorrectionState.NONE) {

            // Trigger when error starts growing again (we passed the target)
            boolean overshoot =
                    (absError > POSITION_TOLERANCE) &&
                            (Math.abs(error) > Math.abs(lastError));

            if (overshoot) {
                correction = CorrectionState.REWIND;

                // REWIND opposite direction of last movement
                rewindDirection = (lastError > 0) ? -1 : 1;
                rewindStartTime = now;

                telemetry.addData("CorrState", "START_REWIND");
                lastError = error;
                return;
            }
        }


        // ==========================================
        //     CORRECTION STATE MACHINE
        // ==========================================

        // ---- REWIND PHASE ----
        if (correction == CorrectionState.REWIND) {

            sorterMotor.setVelocity(600 * rewindDirection);

            // Short rewind, then switch to SNAP
            if (now - rewindStartTime >= 80) {
                sorterMotor.setVelocity(0);

                correction = CorrectionState.SNAP;
                snapStartTime = now;

                // Snap direction: move back toward target
                snapDirection = (targetPos - sorterMotor.getCurrentPosition() > 0) ? 1 : -1;
            }

            telemetry.addData("CorrState", "REWIND");
            lastError = error;
            return;
        }


        // ---- SNAP PHASE ----
        if (correction == CorrectionState.SNAP) {

            sorterMotor.setVelocity(250 * snapDirection);

            boolean timeout = (now - snapStartTime >= 250);

            if (absError <= POSITION_TOLERANCE) {
                // Success: in tolerance → fully done
                sorterMotor.setVelocity(0);
                correction = CorrectionState.NONE;
                moving = false;
            } else if (timeout) {
                // Timed out but still off-target:
                // Stop special correction, let normal PD finish the job
                sorterMotor.setVelocity(0);
                correction = CorrectionState.NONE;
                // moving stays true
            }

            telemetry.addData("CorrState", "SNAP");
            lastError = error;

            // If we cleared correction and still moving, fall through to normal PD
            if (correction != CorrectionState.NONE || !moving) {
                return;
            }
            // else continue into normal PD logic below
        }


        // ==========================================
        //   HARD SETTLE — NEVER STOP OUTSIDE TOLERANCE
        // ==========================================
        if (absError > POSITION_TOLERANCE) {

            stableCount = 0;

            // Full PD control
            int dTerm = (int)(Kd * (error - lastError));
            int vel = (int)(Kp * error + dTerm);

            // Clamp to max speed
            if (vel > MAX_VEL) vel = MAX_VEL;
            if (vel < -MAX_VEL) vel = -MAX_VEL;

            // Slow a bit near target
            if (absError < SLOW_ZONE && Math.abs(vel) > 250) {
                vel = 250 * (error < 0 ? -1 : 1);
            }

            // HARD minimum correction speed (softened)
            if (Math.abs(vel) < 80) {
                vel = 80 * (error < 0 ? -1 : 1);
            }

            sorterMotor.setVelocity(vel);

            telemetry.addData("CorrState", "CORRECTING");
            lastError = error;
            return;
        }


        // ==========================================
        //   INSIDE TOLERANCE → STABILIZATION PHASE
        // ==========================================
        stableCount++;
        if (stableCount >= 3) {
            sorterMotor.setVelocity(0);
            moving = false;

            telemetry.addData("CorrState", "HOLD_DONE");
            lastError = error;
            return;
        }

        // gentle nudge until stable (softened)
        int fineVel = (int)(error * (Kp * 0.25));   // was 0.4
        if (Math.abs(fineVel) < 60)
            fineVel = 60 * (error < 0 ? -1 : 1);

        sorterMotor.setVelocity(fineVel);

        telemetry.addData("CorrState", "HOLD_NUDGE");
        lastError = error;
    }


    public void init_loop() {

        // 1) If homing fault occurred — STOP EVERYTHING
        if (homingFault) {
            sorterMotor.setVelocity(0);
            telemetry.addLine("SORTER HOMING FAILED, SORTER DISABLED!");
            telemetry.addData("Attempts", homingAttempts);
            telemetry.addData("Alpha", opticalSorterHoming.alpha());
            telemetry.update();
            return;
        }

        // 2) If NOT homed yet → keep homing
        if (!sorterHomed) {
            homeSorter();

            telemetry.addData("Homing", "IN PROGRESS");
            telemetry.addData("Attempts", homingAttempts);
            telemetry.addData("Alpha", opticalSorterHoming.alpha());
            telemetry.update();
            return;
        }

        // 3) If homed but not tuned → run auto tuning ONCE
        if (!tuned) {
            autoTuneGains();

            telemetry.addData("Homing", "DONE");
            telemetry.addData("Tuning", "AUTO KD/KP...");
            telemetry.addData("Alpha", opticalSorterHoming.alpha());
            telemetry.update();
            return;
        }

        // 4) ALL SYSTEMS READY — normal update
        update();

        telemetry.addData("Homing", "DONE");
        telemetry.addData("Tuned", tuned);
        telemetry.addData("Alpha", opticalSorterHoming.alpha());
        telemetry.addData("Encoder", sorterMotor.getCurrentPosition());
        telemetry.addData("Kp", Kp);
        telemetry.addData("Kd", Kd);
        telemetry.update();
    }
}
