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

        if (normGreen > 0.05 && normGreen > normRed && normGreen > normBlue) {
            telemetry.addData("Color detected", "GREEN");
            return DetectedColor.GREEN;
        } else if (normBlue > 0.1 && normBlue > normRed && normBlue > normGreen) {
            telemetry.addData("Color detected", "PURPLE");
            return DetectedColor.PURPLE;
        }

        return DetectedColor.UNKNOWN;
    }
}
