
//old code
/*

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.robocol.TelemetryMessage;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp(name = "LinearActuatorTester")
public class ColorSensorTester extends LinearOpMode {

    public void runOpMode() throws InterruptedException {
        NormalizedColorSensor colorSensor;
        /*enum DetectedColor {

            RED,
            BLUE,
            YELLOW,
            PURPLE,
            GREEN,
            UNKNOWN

        }*/
        waitForStart();

        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        while (opModeIsActive()) {

            /*public DetectedColor getDetectedColor(Telemetry telemetry){
                NormalizedRGBA colors = colorSensor.getNormalizedColors(); //returns 4 values

                float normRed, normGreen, normBlue;
                normRed = colors.red / colors.alpha;
                normGreen = colors.green / colors.alpha;
                normBlue = colors.blue / colors.alpha;

                telemetry.addData("red", normRed);
                telemetry.addData("green", normGreen);
                telemetry.addData("blue", normBlue);

                if (normGreen > 0.5 && normRed < 0.3 && normBlue < 0.3) {
                    telemetry.addData("Color detected", "green");
                    return DetectedColor.GREEN;
                } else if (normRed > 0.4 && normBlue > 0.4 && normGreen < 0.3) {
                    telemetry.addData("Color detected", "purple");
                    return DetectedColor.PURPLE;
                }
                return DetectedColor.UNKNOWN;
            }*/
        }
    }

    /*

    NormalizedColorSensor colorSensor;
    public enum DetectedColor {

        RED,
        BLUE,
        YELLOW,
        PURPLE,
        GREEN,
        UNKNOWN

    }

    public void init(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");
    }


    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors= colorSensor.getNormalizedColors(); //returns 4 values

        float normRed, normGreen, normBlue;
        normRed = colors.red / colors.alpha;
        normGreen = colors.green / colors.alpha;
        normBlue = colors.blue / colors.alpha;

        telemetry.addData("red", normRed);
        telemetry.addData("green", normGreen);
        telemetry.addData("blue", normBlue);

        if(normGreen > 0.5 && normRed < 0.3 && normBlue < 0.3) {
            telemetry.addData("Color detected","green");
            return DetectedColor.GREEN;
        }
        else if (normRed > 0.4 && normBlue > 0.4 && normGreen < 0.3) {
            telemetry.addData("Color detected","purple");
            return DetectedColor.PURPLE;
        }
        return DetectedColor.UNKNOWN;



    }
    */

}
*/
//New Code
        package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

@TeleOp(name = "ColorSensorTester")
public class ColorSensorTester extends LinearOpMode {

    private NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        RED,
        BLUE,
        YELLOW,
        PURPLE,
        GREEN,
        UNKNOWN
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize hardware
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color_distance");

        telemetry.addLine("ColorSensorTester ready");
        telemetry.addData("Sensor", "sensor_color_distance");
        telemetry.update();

        // Wait for Play
        waitForStart();

        while (opModeIsActive()) {
            // Read normalized RGBA from sensor
            NormalizedRGBA colors = colorSensor.getNormalizedColors();

            // Avoid division by zero when normalizing
            float alpha = colors.alpha;
            if (alpha <= 0.0001f) {
                alpha = 1.0f;
            }

            // Normalize RGB by alpha (brightness)
            float normRed   = colors.red   / alpha;
            float normGreen = colors.green / alpha;
            float normBlue  = colors.blue  / alpha;

            // Send raw & normalized values to telemetry
            telemetry.addData("raw R G B A", "%.3f  %.3f  %.3f  %.3f",
                    colors.red, colors.green, colors.blue, colors.alpha);
            telemetry.addData("norm R G B", "%.3f  %.3f  %.3f",
                    normRed, normGreen, normBlue);

            // Detect color using a helper method
            DetectedColor detected = getDetectedColor(normRed, normGreen, normBlue);
            telemetry.addData("Detected Color", detected.toString());

            telemetry.update();

            // Let the system breathe a bit
            idle();
        }
    }

    /**
     * Simple rule-based color detection using normalized RGB values.
     * Tweak thresholds to match your physical testing environment.
     */
    private DetectedColor getDetectedColor(float r, float g, float b) {
        // Example thresholds (tune these while testing)
        if (g > 0.5f && r < 0.35f && b < 0.35f) {
            return DetectedColor.GREEN;
        }
        if (r > 0.6f && g < 0.35f && b < 0.35f) {
            return DetectedColor.RED;
        }
        if (b > 0.55f && r < 0.35f && g < 0.35f) {
            return DetectedColor.BLUE;
        }
        if (r > 0.45f && b > 0.45f && g < 0.35f) {
            return DetectedColor.PURPLE;
        }
        if (r > 0.45f && g > 0.45f && b < 0.35f) {
            return DetectedColor.YELLOW;
        }
        return DetectedColor.UNKNOWN;
    }
}
