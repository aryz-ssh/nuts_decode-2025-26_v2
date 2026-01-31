package org.firstinspires.ftc.teamcode.autocode.finalAutos;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Blue Close Leave", group = "Autonomous")
@Config
public class BlueCloseLeave extends LinearOpMode {

    private Follower follower;
    private Paths paths;

    // ================= DASHBOARD TUNING =================

    // ---------- START ----------
    public static double START_X = 21.854;
    public static double START_Y = 123.317;
    public static double START_H = 145;

    // ---------- MID ----------
    public static double MID_X = 37.659;
    public static double MID_Y = 111.707;
    public static double MID_H = 180;

    // ---------- END ----------
    public static double END_X = 17.122;
    public static double END_Y = 109.146;

    @Override
    public void runOpMode() {

        follower = Constants.createFollower(hardwareMap);

        // MUST match first path start
        follower.setStartingPose(new Pose(
                START_X,
                START_Y,
                Math.toRadians(START_H)
        ));

        paths = new Paths(follower);

        waitForStart();

        // -------- PATH 1 --------
        follower.followPath(paths.Path1);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }

        // -------- PATH 2 --------
        follower.followPath(paths.Path2);
        while (opModeIsActive() && follower.isBusy()) {
            follower.update();
        }
    }

    // ================= PATHS =================
    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;

        public Paths(Follower follower) {

            // -------- PATH 1 --------
            Path1 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(START_X, START_Y),
                            new Pose(MID_X, MID_Y)
                    ))
                    .setLinearHeadingInterpolation(
                            Math.toRadians(START_H),
                            Math.toRadians(MID_H)
                    )
                    .build();

            // -------- PATH 2 --------
            Path2 = follower.pathBuilder()
                    .addPath(new BezierLine(
                            new Pose(MID_X, MID_Y),
                            new Pose(END_X, END_Y)
                    ))
                    .setTangentHeadingInterpolation()
                    .build();
        }
    }
}
