package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "PostNut")
public class PostNut extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();

    // ---------- Gamepad ----------
    public static double DEADZONE = 0.05;
    public static double GAMEPAD_TRIGGER_THRESHOLD = 0.25;

    // ---------- Mechanisms ----------
    private Mechanisms mechanisms;
    private MasterDrivetrain drivetrain;
    private AprilTagLimelight limelight;

    // ---------- Intake ----------
    private boolean intakeToggle = false;
    private boolean lastIntakeTrigger = false;

    // ---------- Sorter ----------
    private int selectedPocket = 1;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastX = false;

    private enum SorterMode { NONE, INTAKE, OUTTAKE }
    private SorterMode sorterMode = SorterMode.INTAKE;
    private boolean lastGreenRequest = false;
    private boolean lastPurpleRequest = false;

    // ---------- Outtake ----------
    private boolean outtakeOn = false;
    private boolean lastB = false;

    // ---------- Ramp angle debounce ----------
    private boolean lastRB2 = false;
    private boolean lastLB2 = false;

    // ---------- Outtake speed debounce ----------
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;

    @Override
    public void runOpMode() throws InterruptedException {

        // ---------- Initialize ----------
        mechanisms = new Mechanisms();
        mechanisms.initMechanisms(hardwareMap, telemetry);

        drivetrain = new MasterDrivetrain();
        drivetrain.init(hardwareMap);

        limelight = new AprilTagLimelight(hardwareMap);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // ---------- Main loop ----------
        while (opModeIsActive()) {

            // ---------- Drive ----------
            double y  = applyDeadband(-gamepad1.left_stick_y);
            double x  = applyDeadband(gamepad1.left_stick_x);
            double rx = applyDeadband(gamepad1.right_stick_x);

            // ---------- Auto-align on X button ----------
            rx = limelight.getAutoAlignTurn(gamepad1.x, rx);

            boolean brake = gamepad1.left_trigger > GAMEPAD_TRIGGER_THRESHOLD;
            drivetrain.driveRobotCentric(x, y, rx);

            // ---------- Intake toggle ----------
            boolean intakeTriggerNow = gamepad1.right_trigger > GAMEPAD_TRIGGER_THRESHOLD;
            boolean reversePressed = gamepad1.left_bumper;

            if (intakeTriggerNow && !lastIntakeTrigger) {
                intakeToggle = !intakeToggle;
                if (intakeToggle) mechanisms.engageIntake(1.0, reversePressed);
                else mechanisms.disengageIntake();
            }
            lastIntakeTrigger = intakeTriggerNow;

            if (intakeToggle) {
                mechanisms.engageIntake(1.0, reversePressed);
            }

            // ---------- Gamepad 2 mechanisms ----------
            if (gamepad2.dpad_right && !lastDpadRight) {
                selectedPocket++;
                if (selectedPocket > 3) selectedPocket = 1;
                applySorterModeToPocket();
            }
            if (gamepad2.dpad_left && !lastDpadLeft) {
                selectedPocket--;
                if (selectedPocket < 1) selectedPocket = 3;
                applySorterModeToPocket();
            }
            lastDpadRight = gamepad2.dpad_right;
            lastDpadLeft = gamepad2.dpad_left;

            handleColorRequest(SorterLogicColor.BallColor.GREEN, gamepad2.left_trigger > GAMEPAD_TRIGGER_THRESHOLD);
            handleColorRequest(SorterLogicColor.BallColor.PURPLE, gamepad2.right_trigger > GAMEPAD_TRIGGER_THRESHOLD);

            if (gamepad2.a && !lastA) {
                mechanisms.sorterLogic.markPocketReady(selectedPocket);
                mechanisms.sorterGoToIntake(selectedPocket);
                sorterMode = SorterMode.INTAKE;
            }
            lastA = gamepad2.a;

            if (gamepad2.x && !lastX) {
                mechanisms.sorterGoToOuttake(selectedPocket);
                sorterMode = SorterMode.OUTTAKE;
            }
            lastX = gamepad2.x;

            if (gamepad2.b && !lastB) {
                outtakeOn = !outtakeOn;
                if (outtakeOn) mechanisms.engageOuttake(mechanisms.getManualOuttakeSpeed());
                else mechanisms.disengageOuttake();
            }
            lastB = gamepad2.b;

            if (gamepad2.dpad_up && !lastDpadUp) mechanisms.increaseOuttakeSpeed(0.1);
            if (gamepad2.dpad_down && !lastDpadDown) mechanisms.decreaseOuttakeSpeed(0.1);
            lastDpadUp = gamepad2.dpad_up;
            lastDpadDown = gamepad2.dpad_down;

            if (gamepad2.y) {
                mechanisms.setShotPocket(selectedPocket);
                mechanisms.ejectBall();
            }

            // Ramp angle
            boolean rb2 = gamepad2.right_bumper;
            boolean lb2 = gamepad2.left_bumper;

            if (rb2 && !lastRB2) mechanisms.adjustOuttakeAngle(true, false);
            if (lb2 && !lastLB2) mechanisms.adjustOuttakeAngle(false, true);

            lastRB2 = rb2;
            lastLB2 = lb2;

            mechanisms.updateMechanisms();

            // ---------- Telemetry ----------
            telemetry.addData("Turn Cmd (rx)", "%.2f", rx);
            telemetry.addData("Target Seen", limelight.hasTarget());
            telemetry.addData("Selected Pocket", selectedPocket);
            telemetry.update();
        }
    }

    // ---------- Helper Methods ----------
    private void applySorterModeToPocket() {
        switch (sorterMode) {
            case INTAKE:
                mechanisms.sorterGoToIntake(selectedPocket);
                break;
            case OUTTAKE:
                mechanisms.sorterGoToOuttake(selectedPocket);
                break;
            case NONE:
                break;
        }
    }

    private double applyDeadband(double value) {
        return Math.abs(value) > DEADZONE ? value : 0;
    }

    private void handleColorRequest(SorterLogicColor.BallColor color, boolean pressed) {
        boolean lastRequest = color == SorterLogicColor.BallColor.GREEN ? lastGreenRequest : lastPurpleRequest;
        if (pressed && !lastRequest) {
            Integer pocket = mechanisms.sorterLogic.getPocketWithColor(color);
            if (pocket != null) {
                selectedPocket = pocket;
                sorterMode = SorterMode.OUTTAKE;
                mechanisms.sorterLogic.setCurrentIntakePocket(pocket);
                mechanisms.sorterGoToOuttake(selectedPocket);
            }
        }
        if (color == SorterLogicColor.BallColor.GREEN) lastGreenRequest = pressed;
        else lastPurpleRequest = pressed;
    }

}
