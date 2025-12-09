package org.firstinspires.ftc.teamcode.testers;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorSensorTester Ball Detect NN")
public class ColorSensorTester extends LinearOpMode {

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

        NormalizedRGBA c = colorSensor.getNormalizedColors();

        float alpha = c.alpha;
        float red   = c.red;
        float green = c.green;
        float blue  = c.blue;

        telemetry.addData("Alpha", alpha);
        telemetry.addData("Red", red);
        telemetry.addData("Green", green);
        telemetry.addData("Blue", blue);

        // 1. BALL PRESENCE
        if (alpha < 0.20f) {
            return DetectedColor.UNKNOWN;
        }

        // 2. PURPLE: BLUE CLEARLY DOMINATES
        if (blue > green + 0.001f && blue > red + 0.001f) {
            return DetectedColor.PURPLE;
        }

        // 3. GREEN: GREEN DOMINATES
        if (green > blue + 0.001f && green > red + 0.001f) {
            return DetectedColor.GREEN;
        }

        return DetectedColor.UNKNOWN;
    }

}
