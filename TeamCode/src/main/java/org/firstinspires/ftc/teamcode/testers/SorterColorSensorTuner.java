package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "SorterColorSensorTuner", group = "Debug")
public class SorterColorSensorTuner extends LinearOpMode {

    ColorSensor cs;

    @Override
    public void runOpMode() throws InterruptedException {

        cs = hardwareMap.get(ColorSensor.class, "sorterColorSensor");

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("Red", cs.red());
            telemetry.addData("Green", cs.green());
            telemetry.addData("Blue", cs.blue());
            telemetry.addData("Alpha", cs.alpha());

            telemetry.update();
        }
    }
}
