package org.firstinspires.ftc.teamcode.autocode;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

//import org.firstinspires.ftc.teamcode.Mechanisms;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "AutoCode", group = "Autonomous")
@Configurable
public class PreNut extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;
    private Paths paths;
   // private Mechanisms mechanisms;

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(84.4, 10.4, Math.toRadians(90)));

        paths = new Paths(follower);

      //  mechanisms = new Mechanisms();
      //  mechanisms.initMechanisms(hardwareMap, telemetry);

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        int state = 0;
        while (opModeIsActive()) {

            follower.update();

            switch (state) {

                case 0:
                    follower.followPath(paths.FirstBallsStart);
                    state = 1;
                    break;

                case 1:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.FirstBallsEnd);
                        state = 2;
                    }
                    break;
                case 2:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.FirstBallsShoot);
                        state = 3;
                    }
                    break;
                case 3:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.SecondBallsStart);
                        state = 4;
                    }
                    break;
                case 4:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.SecondBallsEnd);
                        state = 5;
                    }
                    break;
                case 5:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.SecondBallsShoot);
                        state = 6;
                    }
                    break;
                case 6:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.ThirdBallsStart);
                        state = 7;
                    }
                    break;
                case 7:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.ThirdBallsEnd);
                        state = 8;
                    }
                    break;
                case 8:
                    if (!follower.isBusy()) {
                        follower.followPath(paths.ThirdBallsShoot);
                        state = 9;
                    }
                    break;
            }

            panelsTelemetry.debug("State", state);
            panelsTelemetry.debug("X", follower.getPose().getX());
            panelsTelemetry.debug("Y", follower.getPose().getY());
            panelsTelemetry.update(telemetry);
        }
    }

    // ---------------- PATH LIST ----------------
    public static class Paths {

        public PathChain FirstBallsStart;
        public PathChain FirstBallsEnd;
        public PathChain FirstBallsShoot;
        public PathChain SecondBallsStart;
        public PathChain SecondBallsEnd;
        public PathChain SecondBallsShoot;
        public PathChain ThirdBallsStart;
        public PathChain ThirdBallsEnd;
        public PathChain ThirdBallsShoot;

        public Paths(Follower follower) {
            FirstBallsStart = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(49.000, 9.000), new Pose(40.000, 36.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            FirstBallsEnd = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.000, 36.000), new Pose(20.000, 36.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            FirstBallsShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 36.000), new Pose(67.000, 77.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();

            SecondBallsStart = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(67.000, 77.000), new Pose(40.000, 60.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            SecondBallsEnd = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.000, 60.000), new Pose(20.000, 60.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            SecondBallsShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 60.000), new Pose(59.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();

            ThirdBallsStart = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(59.000, 84.000), new Pose(40.000, 84.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(140), Math.toRadians(180))
                    .build();

            ThirdBallsEnd = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(40.000, 84.000), new Pose(20.000, 84.000))
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            ThirdBallsShoot = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(new Pose(20.000, 84.000), new Pose(43.000, 100.000))
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(140))
                    .build();
        }
    }
}