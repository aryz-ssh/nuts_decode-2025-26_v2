package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;

@TeleOp(name = "SorterPocketCalibration", group = "Calibration")
public class SorterPocketCalibration extends LinearOpMode {

    SorterLogic sorter = new SorterLogic();

    @Override
    public void runOpMode() throws InterruptedException {

        sorter.init(hardwareMap, telemetry);

        telemetry.addLine(">> Press INIT to begin homing...");
        telemetry.update();

        // Homing in init loop
        while (!isStarted() && !isStopRequested()) {
            sorter.init_loop();
        }

        waitForStart();

        telemetry.clearAll();
        telemetry.addLine("HOMED!");
        telemetry.addLine("Use:");
        telemetry.addLine("  DPAD UP    = bump forward");
        telemetry.addLine("  DPAD DOWN  = bump backward");
        telemetry.addLine("Align each pocket, write down encoder values.");
        telemetry.addLine("Press STOP to finish.");
        telemetry.update();

        while (opModeIsActive()) {

            double bumpPower = 0.10;

            // Nudge forward
            if (gamepad1.dpad_up) {
                sorter.sorterMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sorter.sorterMotor.setPower(bumpPower);
            }
            // Nudge backward
            else if (gamepad1.dpad_down) {
                sorter.sorterMotor.setMode(com.qualcomm.robotcore.hardware.DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                sorter.sorterMotor.setPower(-bumpPower);
            }
            else {
                sorter.sorterMotor.setPower(0);
            }

            telemetry.addLine("Align pocket and WRITE THIS VALUE DOWN:");
            telemetry.addData("Encoder", sorter.sorterMotor.getCurrentPosition());
            telemetry.update();

            sleep(30);
        }
    }
}
