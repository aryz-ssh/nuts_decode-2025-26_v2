package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.SorterLogicColor;

@TeleOp(name = "Flywheel Tuner (Direct + Sorter + Kicker)", group = "Test")
@Config
public class FlywheelTuner extends OpMode {

    // ================= FLYWHEEL =================
    private DcMotorEx outtakeMotor;
    private FtcDashboard dashboard;
    private TelemetryPacket packet;

    private static final double TICKS_PER_REV_6000 = 28.0;
    private static final double MAX_TICKS_PER_SEC = 2360.0;

    public static double flywheelPower = 0.7;
    private boolean flywheelEnabled = false;

    // ================= PIDF =================
    public static double P = 0.0;
    public static double F = 13.9;

    private final double[] stepSizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    private int stepIndex = 2;

    // ================= SORTER =================
    private SorterLogicColor sorter;

    private enum SorterMode { INTAKE, OUTTAKE }
    private SorterMode sorterMode = SorterMode.INTAKE;
    private int selectedPocket = 1;

    // ================= KICKER =================
    private Servo kickerServo;
    public static double KICKER_REST = 0.45;
    public static double KICKER_FIRE = 0.83;
    public static double KICK_TIME = 0.25;

    private boolean kickerActive = false;
    private ElapsedTime kickerTimer = new ElapsedTime();

    // ================= DEBOUNCE =================
    private boolean lastB = false;
    private boolean lastDpadUp = false;
    private boolean lastDpadDown = false;
    private boolean lastDpadLeft = false;
    private boolean lastDpadRight = false;
    private boolean lastA = false;
    private boolean lastX = false;
    private boolean lastRT = false;

    // ===== DASHBOARD GRAPH VARIABLES =====
    public static double targetVelocity = 0;
    public static double actualVelocity = 0;
    public static double velocityError = 0;
    public static double flywheelEnabledFlag = 0; // 1 or 0 for graph visibility


    @Override
    public void init() {

        // ---------- FLYWHEEL ----------
        outtakeMotor = hardwareMap.get(DcMotorEx.class, "outtakeMotor");
        outtakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        outtakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        outtakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        applyPIDF();

        // ---------- KICKER ----------
        kickerServo = hardwareMap.get(Servo.class, "kickerServo");
        kickerServo.setPosition(KICKER_REST);

        // ---------- SORTER ----------
        sorter = new SorterLogicColor();
        sorter.init(hardwareMap, telemetry);
        sorter.autoAdvanceEnabled = false;

        telemetry.addLine("Flywheel Tuner READY (Direct Control)");
        telemetry.update();

        dashboard = FtcDashboard.getInstance();
    }

    @Override
    public void init_loop() {
        sorter.init_loop();
    }

    @Override
    public void loop() {

        // ==================================================
        // PIDF TUNING (GAMEPAD 1)
        // ==================================================
        if (gamepad1.bWasPressed())
            stepIndex = (stepIndex + 1) % stepSizes.length;

        if (gamepad1.dpadUpWasPressed())    P += stepSizes[stepIndex];
        if (gamepad1.dpadDownWasPressed())  P -= stepSizes[stepIndex];
        if (gamepad1.dpadLeftWasPressed())  F += stepSizes[stepIndex];
        if (gamepad1.dpadRightWasPressed()) F -= stepSizes[stepIndex];

        applyPIDF();

        // ==================================================
        // SORTER POCKET SELECT
        // ==================================================
        if (gamepad2.dpad_right && !lastDpadRight) {
            selectedPocket++;
            if (selectedPocket > 3) selectedPocket = 1;
            applySorterPose();
        }

        if (gamepad2.dpad_left && !lastDpadLeft) {
            selectedPocket--;
            if (selectedPocket < 1) selectedPocket = 3;
            applySorterPose();
        }

        lastDpadRight = gamepad2.dpad_right;
        lastDpadLeft  = gamepad2.dpad_left;

        // ==================================================
        // SORTER POSE
        // ==================================================
        if (gamepad2.a && !lastA) {
            sorterMode = SorterMode.INTAKE;
            sorter.goToIntake(selectedPocket);
        }

        if (gamepad2.x && !lastX) {
            sorterMode = SorterMode.OUTTAKE;
            sorter.goToOuttake(selectedPocket);
        }

        lastA = gamepad2.a;
        lastX = gamepad2.x;

        // ==================================================
        // FLYWHEEL TOGGLE
        // ==================================================
        if (gamepad2.b && !lastB) {
            flywheelEnabled = !flywheelEnabled;
        }
        lastB = gamepad2.b;

        // ==================================================
        // FLYWHEEL POWER
        // ==================================================
        if (gamepad2.dpad_up && !lastDpadUp)
            flywheelPower = Math.min(1.0, flywheelPower + 0.1);

        if (gamepad2.dpad_down && !lastDpadDown)
            flywheelPower = Math.max(0.0, flywheelPower - 0.1);

        lastDpadUp = gamepad2.dpad_up;
        lastDpadDown = gamepad2.dpad_down;

        // ==================================================
        // APPLY FLYWHEEL (NO GATES)
        // ==================================================
        if (flywheelEnabled)
            outtakeMotor.setVelocity(flywheelPower * MAX_TICKS_PER_SEC);
        else
            outtakeMotor.setVelocity(0);

        // ==================================================
        // KICKER (RIGHT TRIGGER)
        // ==================================================
        boolean rt = gamepad2.right_trigger > 0.5;

        if (rt && !lastRT && !kickerActive) {
            kickerActive = true;
            kickerTimer.reset();
            kickerServo.setPosition(KICKER_FIRE);
        }

        if (kickerActive && kickerTimer.seconds() > KICK_TIME) {
            kickerServo.setPosition(KICKER_REST);
            kickerActive = false;
        }

        lastRT = rt;

        // ==================================================
        // UPDATE SORTER
        // ==================================================
        sorter.update();

        targetVelocity = flywheelEnabled
                ? flywheelPower * MAX_TICKS_PER_SEC
                : 0;

        actualVelocity = outtakeMotor.getVelocity();
        velocityError  = targetVelocity - actualVelocity;
        flywheelEnabledFlag = flywheelEnabled ? 1.0 : 0.0;

        packet = new TelemetryPacket();
        packet.put("targetVelocity", targetVelocity);
        packet.put("actualVelocity", actualVelocity);
        packet.put("velocityError", velocityError);
        packet.put("flywheelEnabled", flywheelEnabledFlag);

        dashboard.sendTelemetryPacket(packet);

        // ==================================================
        // TELEMETRY
        // ==================================================
        telemetry.addData("Pocket", selectedPocket);
        telemetry.addData("Sorter Mode", sorterMode);
        telemetry.addData("Sorter Pos", sorter.getCurrentPos());
        telemetry.addData("Sorter Target", sorter.getTargetPos());
        telemetry.addData("Sorter State", sorter.getDebugCorrState());

        telemetry.addLine("--- Flywheel ---");
        telemetry.addData("Enabled", flywheelEnabled);
        telemetry.addData("Power", "%.2f", flywheelPower);
        telemetry.addData("Target Vel", "%.0f",
                flywheelPower * MAX_TICKS_PER_SEC);
        telemetry.addData("Actual Vel", "%.0f",
                outtakeMotor.getVelocity());

        telemetry.addLine("--- Kicker ---");
        telemetry.addData("Active", kickerActive);

        telemetry.addLine("--- PIDF ---");
        telemetry.addData("P", "%.6f", P);
        telemetry.addData("F", "%.6f", F);
        telemetry.addData("Step", stepSizes[stepIndex]);

        telemetry.update();
    }

    private void applySorterPose() {
        if (sorterMode == SorterMode.INTAKE)
            sorter.goToIntake(selectedPocket);
        else
            sorter.goToOuttake(selectedPocket);
    }

    private void applyPIDF() {
        outtakeMotor.setPIDFCoefficients(
                DcMotor.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(P, 0, 0, F)
        );
    }
}
