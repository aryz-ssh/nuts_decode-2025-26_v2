package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class PedroMecanumMathOnly extends Mecanum {

    public PedroMecanumMathOnly(HardwareMap hw, MecanumConstants constants) {
        super(hw, constants);
    }

    /** Disable all motor output **/
    @Override
    public void runDrive(double[] drivePowers) {
        // Pedro math-only – motors controlled externally
    }

    /** Disable Pedro's stop behavior **/
    @Override
    public void breakFollowing() {
        // disable hardware float/stop
    }

    /** Disable teleop drive mode switching (float/brake) **/
    @Override
    public void startTeleopDrive() {}

    @Override
    public void startTeleopDrive(boolean brakeMode) {}
}
