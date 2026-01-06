package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroMotorIO {

    // Front wheels — ALWAYS constant
    public static double FRONT_SCALE = 1.0;

    // Rear wheels — minimum scale at pure strafe
    public static double REAR_MIN_SCALE = 0.54;

    // Strafe fade-in control (0..1 clarity)
    public static double STRAFE_FADE_START = 0.25;
    public static double STRAFE_FADE_END   = 0.85;

    private final DcMotorEx fl, bl, fr, br;

    public PedroMotorIO(HardwareMap hw) {
        fl = hw.get(DcMotorEx.class, "leftFront");
        bl = hw.get(DcMotorEx.class, "leftBack");
        fr = hw.get(DcMotorEx.class, "rightFront");
        br = hw.get(DcMotorEx.class, "rightBack");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        fl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        fr.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        bl.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
        br.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }

    public void setPowers(double flp, double blp, double frp, double brp) {
        fl.setPower(flp);
        bl.setPower(blp);
        fr.setPower(frp);
        br.setPower(brp);
    }

    public void setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior behavior) {
        fl.setZeroPowerBehavior(behavior);
        bl.setZeroPowerBehavior(behavior);
        fr.setZeroPowerBehavior(behavior);
        br.setZeroPowerBehavior(behavior);
    }

    public void stop() {
        setPowers(0, 0, 0, 0);
    }
}
