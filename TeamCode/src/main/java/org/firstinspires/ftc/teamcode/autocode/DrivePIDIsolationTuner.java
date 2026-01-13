package org.firstinspires.ftc.teamcode.autocode;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Config
@TeleOp(name = "Drive PID Isolation Tuner", group = "Pedro Pathing")
public class DrivePIDIsolationTuner extends OpMode {

    // ===== DASHBOARD TOGGLES =====
    public static boolean ENABLE_HEADING = false;
    public static boolean ENABLE_TRANSLATIONAL = false;

    public static double DISTANCE = 30.0; // inches

    private Follower follower;
    private boolean forward = true;

    private Path forwardPath;
    private Path backwardPath;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);

        follower.setStartingPose(new Pose(72, 72, 0));

        forwardPath = new Path(
                new BezierLine(
                        new Pose(72, 72),
                        new Pose(72 + DISTANCE, 72)
                )
        );
        forwardPath.setConstantHeadingInterpolation(0);

        backwardPath = new Path(
                new BezierLine(
                        new Pose(72 + DISTANCE, 72),
                        new Pose(72, 72)
                )
        );
        backwardPath.setConstantHeadingInterpolation(0);
    }

    @Override
    public void start() {
        applyPIDState();
        follower.followPath(forwardPath);
    }

    @Override
    public void loop() {
        applyPIDState();

        follower.update();

        if (!follower.isBusy()) {
            if (forward) {
                follower.followPath(backwardPath);
            } else {
                follower.followPath(forwardPath);
            }
            forward = !forward;
        }

        telemetry.addData("Forward", forward);
        telemetry.addData("Heading PID", ENABLE_HEADING);
        telemetry.addData("Translational PID", ENABLE_TRANSLATIONAL);
        telemetry.update();
    }

    private void applyPIDState() {
        follower.deactivateAllPIDFs();

        // Drive PID is ALWAYS ON for this tuner
        follower.activateDrive();

        if (ENABLE_HEADING) {
            follower.activateHeading();
        }

        if (ENABLE_TRANSLATIONAL) {
            follower.activateTranslational();
        }
    }
}
