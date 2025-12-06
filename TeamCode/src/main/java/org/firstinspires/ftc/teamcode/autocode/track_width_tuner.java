package org.firstinspires.ftc.teamcode.autocode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.roadrunner.ftc.OverflowEncoder;
import com.acmerobotics.roadrunner.ftc.RawEncoder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "TrackWidth Tuner", group = "Tuning")
public class track_width_tuner extends LinearOpMode {

    private DcMotorEx fl, fr, bl, br;

    private OverflowEncoder leftOdo, rightOdo;

    public static double TURN_POWER = 0.25;
    public static int ROTATION_TICKS = 3000;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        fl = hardwareMap.get(DcMotorEx.class, "leftFront");
        bl = hardwareMap.get(DcMotorEx.class, "leftBack");
        fr = hardwareMap.get(DcMotorEx.class, "rightFront");
        br = hardwareMap.get(DcMotorEx.class, "rightBack");

        fl.setDirection(DcMotorSimple.Direction.REVERSE);
        bl.setDirection(DcMotorSimple.Direction.REVERSE);

        leftOdo  = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "rightBack")));
        rightOdo = new OverflowEncoder(new RawEncoder(hardwareMap.get(DcMotorEx.class, "leftBack")));

        leftOdo.setDirection(DcMotorSimple.Direction.FORWARD);
        rightOdo.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("TrackWidth Tuner Ready");
        telemetry.addLine("Press START to rotate robot.");
        telemetry.update();

        waitForStart();

        // --------------------------
        // Capture initial positions
        // --------------------------
        double leftStart  = leftOdo.getPositionAndVelocity().position;
        double rightStart = rightOdo.getPositionAndVelocity().position;

        // --------------------------
        // Start rotation (clockwise)
        // --------------------------
        fl.setPower(TURN_POWER);
        bl.setPower(TURN_POWER);
        fr.setPower(-TURN_POWER);
        br.setPower(-TURN_POWER);

        while (opModeIsActive()) {

            double leftNow  = leftOdo.getPositionAndVelocity().position;
            double rightNow = rightOdo.getPositionAndVelocity().position;

            double leftDelta  = leftNow - leftStart;
            double rightDelta = rightNow - rightStart;

            telemetry.addData("Left Δ", leftDelta);
            telemetry.addData("Right Δ", rightDelta);
            telemetry.update();

            if (Math.abs(leftDelta) > ROTATION_TICKS &&
                    Math.abs(rightDelta) > ROTATION_TICKS) {
                break;
            }
        }

        // Stop motion
        fl.setPower(0);
        bl.setPower(0);
        fr.setPower(0);
        br.setPower(0);

        // --------------------------
        // Compute distances
        // --------------------------
        double leftDist  = (leftOdo.getPositionAndVelocity().position - leftStart)
                * Constants.localizerConstants.turnTicksToInches;

        double rightDist = (rightOdo.getPositionAndVelocity().position - rightStart)
                * Constants.localizerConstants.turnTicksToInches;

        double delta = rightDist - leftDist;

        // --------------------------
        // Compute track width
        // --------------------------
        double trackWidth = delta / (2 * Math.PI);

        // --------------------------
        // Output Result
        // --------------------------
        while (opModeIsActive()) {
            telemetry.addLine("=== TRACK WIDTH CALCULATED ===");
            telemetry.addData("Left Dist (in)", leftDist);
            telemetry.addData("Right Dist (in)", rightDist);
            telemetry.addData("Delta (in)", delta);
            telemetry.addData("TRACK WIDTH =", trackWidth);
            telemetry.addLine("Put this value in MasterDrivetrain.TRACK_WIDTH");
            telemetry.update();
        }
    }
}
