package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "SorterHomingSensorTest", group = "Calibration")
public class SorterHomingSensorTest extends LinearOpMode {

    ColorSensor opticalSorterHoming;

    @Override
    public void runOpMode() throws InterruptedException {

        opticalSorterHoming = hardwareMap.get(ColorSensor.class, "opticalSorterHoming");

        telemetry.addLine("Rotate the carousel by hand. Watch the numbers.");
        telemetry.addLine("Press PLAY when ready.");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Alpha", opticalSorterHoming.alpha());
            telemetry.addData("Red", opticalSorterHoming.red());
            telemetry.addData("Green", opticalSorterHoming.green());
            telemetry.addData("Blue", opticalSorterHoming.blue());
            telemetry.update();

            sleep(40);
        }
    }
}
