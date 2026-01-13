package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.ftc.localization.localizers.ThreeWheelIMULocalizer;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Wraps heading to [-pi, pi] so Pedro never sees accumulated IMU angles.
 */
public class WrappedThreeWheelIMULocalizer extends ThreeWheelIMULocalizer {

    public WrappedThreeWheelIMULocalizer(HardwareMap hardwareMap,
                                         ThreeWheelIMUConstants constants) {
        super(hardwareMap, constants);
    }

    @Override
    public Pose getPose() {
        Pose p = super.getPose();

        double h = wrapRadians(p.getHeading());

        return new Pose(p.getX(), p.getY(), h);
    }

    /** Wrap angle to [-pi, pi] */
    private double wrapRadians(double angle) {
        while (angle > Math.PI) angle -= 2.0 * Math.PI;
        while (angle < -Math.PI) angle += 2.0 * Math.PI;
        return angle;
    }
}
