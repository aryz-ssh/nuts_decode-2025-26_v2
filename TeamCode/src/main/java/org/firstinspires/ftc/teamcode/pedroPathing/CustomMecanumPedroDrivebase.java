package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.Drivetrain;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MasterDrivetrain;

public class CustomMecanumPedroDrivebase extends Drivetrain {

    private final PedroMecanumMathOnly pedro;
    private final PedroMotorIO motors;
//    public static double lastTranslMag = 0.0;
//    public static double lastRotMag = 0.0;
//    public static double lastOrbitScale = 1.0;

    public CustomMecanumPedroDrivebase(HardwareMap hw, MecanumConstants constants) {
        pedro = new PedroMecanumMathOnly(hw, constants);
        motors = new PedroMotorIO(hw);
    }

    @Override
    public double[] calculateDrive(Vector corrective, Vector heading, Vector pathing, double robotHeading) {
        return pedro.calculateDrive(corrective, heading, pathing, robotHeading);
    }

    @Override
    public void runDrive(double[] p) {
        // Pedro order: [fl, bl, fr, br]
        double fl = p[0];
        double bl = p[1];
        double fr = p[2];
        double br = p[3];

        // 1) wheel -> chassis
        double vx = (fl + fr + bl + br) / 4.0;
        double vy = (-fl + fr + bl - br) / 4.0;
        double om = (-fl + fr - bl + br) / 4.0;

        // 2) single counter-rotation K (rear scale acknowledged)
        double kRear = (1.0 - PedroWheelTuning.REAR_SCALE) / 2.0;
        om += (kRear + PedroWheelTuning.STRAFE_YAW_K) * vy;

        // 3) chassis -> wheel
        double nfl = vx - vy - om;
        double nfr = vx + vy + om;
        double nbl = vx + vy - om;
        double nbr = vx - vy + om;

        // 4) normalize once
        double max = Math.max(
                Math.max(Math.abs(nfl), Math.abs(nfr)),
                Math.max(Math.abs(nbl), Math.abs(nbr))
        );
        if (max > 1.0) {
            nfl /= max;
            nfr /= max;
            nbl /= max;
            nbr /= max;
        }

        // 5) rear scaling LAST
        nbl *= PedroWheelTuning.REAR_SCALE;
        nbr *= PedroWheelTuning.REAR_SCALE;

        motors.setPowers(nfl, nbl, nfr, nbr);
    }


    @Override
    public void breakFollowing() {
        motors.stop();
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
        motors.setZeroPowerBehavior(
                behavior == DcMotor.ZeroPowerBehavior.BRAKE
                        ? DcMotorEx.ZeroPowerBehavior.BRAKE
                        : DcMotorEx.ZeroPowerBehavior.FLOAT
        );
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
