package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorSensorTester")
public class ColorTester extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        telemetry.addLine("Color Sensor Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            float normRed = colors.red / colors.alpha;
            float normGreen = colors.green / colors.alpha;
            float normBlue = colors.blue / colors.alpha;

            telemetry.addData("Red", normRed);
            telemetry.addData("Green", normGreen);
            telemetry.addData("Blue", normBlue);
            sleep(100);
        }
    }
}
