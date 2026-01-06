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

        // ---------- STRAFE CLARITY (wheel space) ----------
        double forward = Math.abs(fl + fr + bl + br);
        double lateral = Math.abs(fl + br - fr - bl);

        double clarity = lateral / (forward + lateral + 1e-6); // 0..1

        // ---------- LINEAR REAR FADE ----------
        double t = (clarity - PedroWheelTuning.STRAFE_FADE_START)
                / (PedroWheelTuning.STRAFE_FADE_END - PedroWheelTuning.STRAFE_FADE_START);

        t = Math.max(0.0, Math.min(1.0, t));

        double rearFade = 1.0 + (PedroWheelTuning.BL - 1.0) * t;
        // NOTE: BL and BR are already equal — intentional

        // ---------- APPLY BASE SCALE + FADE ----------
        fl *= PedroWheelTuning.FL;
        fr *= PedroWheelTuning.FR;

        bl *= PedroWheelTuning.BL_BASE * PedroWheelTuning.BL * rearFade;
        br *= PedroWheelTuning.BR_BASE * PedroWheelTuning.BR * rearFade;

        // ---------- NORMALIZE ----------
        double max = Math.max(1.0,
                Math.max(Math.abs(fl),
                        Math.max(Math.abs(bl),
                                Math.max(Math.abs(fr), Math.abs(br)))));

        motors.setPowers(
                fl / max,
                bl / max,
                fr / max,
                br / max
        );
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
