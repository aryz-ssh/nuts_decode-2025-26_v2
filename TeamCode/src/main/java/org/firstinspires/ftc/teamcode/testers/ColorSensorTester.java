
package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorSensorTester")
public class ColorSensorTester extends LinearOpMode {

    // Enum for detected colors
    public enum DetectedColor {
        PURPLE, GREEN, UNKNOWN
    }

    private NormalizedColorSensor colorSensor;

    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sorterColorSensor");

        telemetry.addLine("Color Sensor Ready");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            DetectedColor detectedColor = getDetectedColor();
            telemetry.addData("Detected Color", detectedColor);
            telemetry.update();
            sleep(100);
        }
    }

    public DetectedColor getDetectedColor() {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        float normRed = colors.red / colors.alpha;
        float normGreen = colors.green / colors.alpha;
        float normBlue = colors.blue / colors.alpha;

        telemetry.addData("Red", normRed);
        telemetry.addData("Green", normGreen);
        telemetry.addData("Blue", normBlue);

        if (normGreen >= 0.0030 && normGreen <= 0.0055 && normGreen >= 0.01 && normGreen <= 0.0130 && normBlue >= 0.0080 && normBlue <= 0.015) {
            telemetry.addData("Color detected", "GREEN");
            return DetectedColor.GREEN;
            //do 2nd thresh
        } else if (normGreen >= 0.0050 && normGreen <= 0.0055 && normGreen >= 0.01 && normGreen <= 0.0130 && normBlue >= 0.0080 && normBlue <= 0.015) {
            telemetry.addData("Color detected", "PURPLE");
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }
}
