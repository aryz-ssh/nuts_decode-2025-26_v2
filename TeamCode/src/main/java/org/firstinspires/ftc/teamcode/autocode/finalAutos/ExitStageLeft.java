package org.firstinspires.ftc.teamcode.autocode.finalAutos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Leave Sideways Left", group = "Autonomous")
@Config
public class ExitStageLeft extends LinearOpMode {

    private Follower follower;
    private Paths paths;

    // ================= DASHBOARD TUNING =================

    // Start pose
    public static double START_X = 56.2;
    public static double START_Y = 8.0;
    public static double START_H = 90;

    // End pose
    public static double END_X = 36.2;
    public static double END_Y = 8.0;
    public static double END_H = 90;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        // Start pose (dashboard-tunable)
        follower.setStartingPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_H)
        ));

        paths = new Paths(follower);

        waitForStart();

        follower.followPath(paths.Path1);

        // Let Pedro finish
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ---------------- PATHS ----------------
    public static class Paths {
        public PathChain Path1;

        public Paths(Follower follower) {
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(START_X, START_Y),
                            new Pose(END_X, END_Y)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(START_H),
                            Math.toRadians(END_H)
                    )
                    .build();
        }
    }
}
