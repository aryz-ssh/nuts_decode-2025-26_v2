package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class StatusLED_RGB {

    private final Servo led;
    public static double OFF     = 0.00;   // 500 µs
    public static double RED     = 0.28;  // 1100 µs
    public static double ORANGE  = 0.333;  // 1200 µs
    public static double YELLOW  = 0.388;  // 1300 µs
    public static double GREEN   = 0.500;  // 1500 µs
    public static double CYAN    = 0.555;  // 1600 µs (Azure)
    public static double BLUE    = 0.611;  // 1700 µs (DO NOT CHANGE)
    public static double PURPLE  = 0.722;  // 1900 µs
    public static double WHITE   = 1.000;  // 2500 µs

    private LEDState currentState = LEDState.OFF;

    public enum LEDState {
        OFF,
        WHITE,
        GREEN,
        PURPLE,
        CYAN,
        YELLOW,
        RED,
        ORANGE,
        BLUE
    }

    public StatusLED_RGB(Servo led) {
        this.led = led;
        setState(LEDState.OFF);
    }

    public void setState(LEDState state) {
        if (state == currentState) return;

        currentState = state;

        switch (state) {
            case OFF:    led.setPosition(OFF); break;
            case RED:    led.setPosition(RED); break;
            case ORANGE: led.setPosition(ORANGE); break;
            case YELLOW: led.setPosition(YELLOW); break;
            case GREEN:  led.setPosition(GREEN); break;
            case CYAN: led.setPosition(CYAN); break;
            case BLUE:   led.setPosition(BLUE); break;
            case PURPLE: led.setPosition(PURPLE); break;
            case WHITE:  led.setPosition(WHITE); break;
        }
    }

    public LEDState getState() {
        return currentState;
    }
}
