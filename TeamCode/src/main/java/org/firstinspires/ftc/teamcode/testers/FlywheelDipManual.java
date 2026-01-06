package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name = "Flywheel Dip Test (Manual)", group = "Test")
@Config
public class FlywheelDipManual extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private DcMotorEx outtakeMotor;
    private Servo kickerServo;

    // ---------------- TUNABLES ----------------
    public static double KICKER_REST = 0.45;
    public static double KICKER_FIRE = 0.83;
    public static double KICK_TIME = 0.25;

    public static double AT_SPEED_PERCENT = 0.95;   // 95% of target velocity

    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC_6000 =
            (TICKS_PER_REV_6000 * 6000) / 60.0;

    // ---------------- STATE ----------------
    private double shooterPower = 0.5;
    private boolean shooterEnabled = false;

    private boolean kickerActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();

    // Flywheel state
    private boolean flywheelAtSpeed = false;

    // ---------------- DIP CAPTURE ----------------
    private boolean shotInProgress = false;
    private ElapsedTime shotTimer = new ElapsedTime();

    private double baselineVel = 0;
    private double minVel = Double.MAX_VALUE;

    private double lastDipPercent = 0;
    private double lastRecoveryTime = 0;

    // ---------------- DEBOUNCE ----------------
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastA = false;

    @Override
    public void runOpMode() {

        // ---------------- INIT HARDWARE ----------------
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");

        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        kickerServo.setPosition(KICKER_REST);

        telemetry.addLine("Flywheel Dip Test Ready");
        telemetry.addLine("DPAD ±0.1 | A = Flywheel | RT = Kick");
        telemetry.update();

        waitForStart();

        // ---------------- MAIN LOOP ----------------
        while (opModeIsActive()) {

            // ================= DPAD POWER CONTROL (DEBOUNCED) =================
            boolean dpadUp = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            if (dpadUp && !lastDpadUp) {
                shooterPower = Math.min(1.0, shooterPower + 0.1);
            }

            if (dpadDown && !lastDpadDown) {
                shooterPower = Math.max(0.0, shooterPower - 0.1);
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // ================= FLYWHEEL TOGGLE =================
            boolean aPressed = gamepad1.a;
            if (aPressed && !lastA) {
                shooterEnabled = !shooterEnabled;
            }
            lastA = aPressed;

            // ================= APPLY FLYWHEEL =================
            double targetVel = shooterEnabled
                    ? shooterPower * MAX_TICKS_PER_SEC_6000
                    : 0;

            if (shooterEnabled) {
                outtakeMotor.setVelocity(targetVel);
            } else {
                outtakeMotor.setVelocity(0);
            }

            double currentVel = outtakeMotor.getVelocity();

            // ================= AT SPEED DETECTION =================
            flywheelAtSpeed =
                    shooterEnabled &&
                            targetVel > 200 &&
                            currentVel >= AT_SPEED_PERCENT * targetVel;

            // ================= KICKER CONTROL =================
            if (gamepad1.right_trigger > 0.5 && !kickerActive) {
                kickerActive = true;
                kickerTimer.reset();
                kickerServo.setPosition(KICKER_FIRE);

                // ---- START SHOT WINDOW ----
                if (shooterEnabled) {
                    shotInProgress = true;
                    baselineVel = currentVel;
                    minVel = currentVel;
                    shotTimer.reset();
                }
            }

            if (kickerActive && kickerTimer.seconds() > KICK_TIME) {
                kickerServo.setPosition(KICKER_REST);
                kickerActive = false;
            }

            // ================= DIP TRACKING =================
            if (shotInProgress) {

                minVel = Math.min(minVel, currentVel);

                // End shot window after recovery or timeout
                if (shotTimer.seconds() > 0.8 || currentVel >= 0.95 * baselineVel) {

                    double drop = baselineVel - minVel;
                    lastDipPercent = (baselineVel > 0)
                            ? drop / baselineVel
                            : 0;

                    lastRecoveryTime = shotTimer.seconds();
                    shotInProgress = false;
                }
            }

            // ================= TELEMETRY =================
            telemetry.addData("Flywheel Enabled", shooterEnabled);
            telemetry.addData("Shooter Power", "%.1f", shooterPower);
            telemetry.addData("Target Vel", "%.0f", targetVel);
            telemetry.addData("Current Vel", "%.0f", currentVel);
            telemetry.addData("Flywheel @ Speed", flywheelAtSpeed);
            telemetry.addData("Baseline Vel", "%.0f", baselineVel);
            telemetry.addData("Min Vel", "%.0f", minVel);
            telemetry.addData("Dip %", "%.1f%%", lastDipPercent * 100.0);
            telemetry.addData("Recovery Time (s)", "%.2f", lastRecoveryTime);
            telemetry.addData("Kicker Active", kickerActive);

            telemetry.addLine(
                    flywheelAtSpeed ? "FLYWHEEL READY" : "FLYWHEEL SPINNING"
            );

            telemetry.update();
        }
    }
}
