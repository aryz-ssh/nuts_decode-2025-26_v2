package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp(name = "Sorter Logic Test with Color")
public class SorterLogicPositionTest extends LinearOpMode {

    SorterLogicV2_indexing_experiment sorter;

    @Override
    public void runOpMode() {

        sorter = new SorterLogicV2_indexing_experiment();
        sorter.init(hardwareMap, telemetry);

        telemetry.addLine("Sorter Logic V2 color Test");
        telemetry.addLine(">>> Waiting for start...");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // Update logic + sensors
            sorter.update();
            sorter.updateSensors();

            // Pocket switching with D-Pad
            if (gamepad1.dpad_up) sorter.goToIntake(1);
            if (gamepad1.dpad_left) sorter.goToIntake(2);
            if (gamepad1.dpad_right) sorter.goToIntake(3);

            if (gamepad1.x) sorter.goToOuttake(1);
            if (gamepad1.y) sorter.goToOuttake(2);
            if (gamepad1.b) sorter.goToOuttake(3);

            // --- TELEMETRY ---
            telemetry.addLine("===== Sorter Test =====");
            telemetry.addData("Curr Pos", sorter.getCurrentPos());
            telemetry.addData("Target Pos", sorter.getTargetPos());

            telemetry.addLine();
            telemetry.addData("Alpha", sorter.getAlpha());
            telemetry.addData("R", sorter.getRed());
            telemetry.addData("G", sorter.getGreen());
            telemetry.addData("B", sorter.getBlue());

            telemetry.addLine();
            telemetry.addData("Ball Present?", sorter.isBallPresent());
            telemetry.addData("Stable Color", sorter.detectBallColor());

            telemetry.update();
        }
    }
}
