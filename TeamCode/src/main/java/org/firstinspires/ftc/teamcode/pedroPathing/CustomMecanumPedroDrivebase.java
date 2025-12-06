package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MasterDrivetrain;

public class CustomMecanumPedroDrivebase extends Drivetrain {

    private final PedroMecanumMathOnly pedro;
    private final MasterDrivetrain master;

    public CustomMecanumPedroDrivebase(HardwareMap hw, MecanumConstants constants) {

        pedro = new PedroMecanumMathOnly(hw, constants);

        master = new MasterDrivetrain();
        master.init(hw);
    }

    @Override
    public double[] calculateDrive(Vector corrective, Vector heading, Vector pathing, double robotHeading) {
        return pedro.calculateDrive(corrective, heading, pathing, robotHeading);
    }

    @Override
    public void runDrive(double[] drivePowers) {
        master.runAutoDrive(drivePowers);
    }

    @Override
    public void breakFollowing() {
        master.stop();
    }

    @Override
    public void startTeleopDrive() {
        // EXACT DEFAULT PEDRO BEHAVIOR:
        if (pedro.constants.useBrakeModeInTeleOp) {
            setZeroPowerAll(DcMotor.ZeroPowerBehavior.BRAKE);
        }
    }

    @Override
    public void startTeleopDrive(boolean brakeMode) {
        DcMotor.ZeroPowerBehavior b =
                brakeMode ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;

        setZeroPowerAll(b);
    }

    private void setZeroPowerAll(DcMotor.ZeroPowerBehavior behavior) {
        master.frontLeft.setZeroPowerBehavior(behavior);
        master.frontRight.setZeroPowerBehavior(behavior);
        master.backLeft.setZeroPowerBehavior(behavior);
        master.backRight.setZeroPowerBehavior(behavior);
    }

    @Override
    public void updateConstants() {
        pedro.updateConstants();
    }

    @Override public double xVelocity() { return pedro.xVelocity(); }
    @Override public double yVelocity() { return pedro.yVelocity(); }

    @Override public void setXVelocity(double x) { pedro.setXVelocity(x); }
    @Override public void setYVelocity(double y) { pedro.setYVelocity(y); }

    @Override public double getVoltage() { return pedro.getVoltage(); }

    @Override public String debugString() { return "CustomMecanumPedroDrivebase"; }
}
