package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "Outtake Beam Break Status LED", group = "Test")
public class BeamBreakSensorTest extends OpMode {

    private DigitalChannel outtakeBeamBreak;
    private Servo statusLED;

    // ===== GoBILDA RGB Indicator values =====
    public static double LED_OFF = 0.0;   // NOT 0.0
    public static double LED_RED = 0.29;  // from GoBILDA chart

    @Override
    public void init() {
        outtakeBeamBreak = hardwareMap.get(DigitalChannel.class, "outtakeBeamBreak");
        statusLED = hardwareMap.servo.get("statusLED");

        outtakeBeamBreak.setMode(DigitalChannel.Mode.INPUT);

        // Force full servo range (important)
        statusLED.scaleRange(0.0, 1.0);
        statusLED.setDirection(Servo.Direction.FORWARD);

        statusLED.setPosition(LED_OFF);
    }

    @Override
    public void loop() {

        boolean objectDetected = !outtakeBeamBreak.getState();

        if (objectDetected) {
            statusLED.setPosition(LED_RED);
        } else {
            statusLED.setPosition(LED_OFF);
        }

        telemetry.addData("Beam Break Raw", outtakeBeamBreak.getState());
        telemetry.addData("Detected", objectDetected);
        telemetry.addData("LED Pos", objectDetected ? LED_RED : LED_OFF);
        telemetry.update();
    }
}
