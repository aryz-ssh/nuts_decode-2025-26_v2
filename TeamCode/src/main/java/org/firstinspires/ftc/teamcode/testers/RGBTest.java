package org.firstinspires.ftc.teamcode.test;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Config
@TeleOp(name = "RGB Indicator Rainbow Test", group = "Test")
public class RGBTest extends OpMode {

    // ===== Dashboard tunables =====
    public static double MIN_POS = 0.00;
    public static double MAX_POS = 1.00;
    public static double STEP = 0.002;   // smaller = smoother, slower
    public static int LOOP_DELAY_MS = 10;

    private Servo rgb;
    private double pos = 0.0;
    private boolean increasing = true;

    @Override
    public void init() {
        rgb = hardwareMap.servo.get("statusLED");
        rgb.setPosition(MIN_POS);
    }

    @Override
    public void loop() {

        rgb.setPosition(pos);

        if (increasing) {
            pos += STEP;
            if (pos >= MAX_POS) {
                pos = MAX_POS;
                increasing = false;
            }
        } else {
            pos -= STEP;
            if (pos <= MIN_POS) {
                pos = MIN_POS;
                increasing = true;
            }
        }

        // Simple timing control (FTC-safe)
        try {
            Thread.sleep(LOOP_DELAY_MS);
        } catch (InterruptedException ignored) {}
    }
}
