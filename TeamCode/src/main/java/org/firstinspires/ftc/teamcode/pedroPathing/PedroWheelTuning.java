package org.firstinspires.ftc.teamcode.pedroPathing;
import com.acmerobotics.dashboard.config.Config;

@Config
public class PedroWheelTuning {
    // -------- CONSTANT BASE SCALES (ALWAYS APPLIED) --------
    public static double FL = 0.9;
    public static double FR = 1.00;
    public static double BL = 0.7;   // your known-good value
    public static double BR = 0.7;

    public static double BL_BASE = 0.8;
    public static double BR_BASE = 1.00;

    // -------- STRAFE FADE (DYNAMIC, REAR ONLY) --------
    // clarity = lateral / (lateral + forward)
    public static double STRAFE_FADE_START = 0.25; // begin rear fade
    public static double STRAFE_FADE_END   = 0.85; // full strafe
}
