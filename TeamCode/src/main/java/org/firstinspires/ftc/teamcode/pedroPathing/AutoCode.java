package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "Pedro Pathing Autonomous", group = "Autonomous")
@Configurable // Panels
public class AutoCode extends OpMode {

    private static final Pose pose1 = new Pose(56.000, 8.000);
    private static final Pose pose2 = new Pose(39.988, 35.456);
    private static final Pose pose3 = new Pose(39.988, 35.456);
    private static final Pose pose4 = new Pose(15.843, 35.674);
    private static final Pose pose5 = new Pose(15.843, 35.674);
    private static final Pose pose6 = new Pose(50.429, 35.456);
    private static final Pose pose7 = new Pose(50.429, 35.456);
    private static final Pose pose8 = new Pose(59.565, 11.094);
    private static final Pose pose9 = new Pose(59.565, 11.094);
    private static final Pose pose10 = new Pose(41.329, 60.036);
    private static final Pose pose11 = new Pose(41.329, 60.036);
    private static final Pose pose12 = new Pose(16.314, 59.819);
    private static final Pose pose13 = new Pose(16.314, 59.819);
    private static final Pose pose14 = new Pose(50.429, 60.036);
    private static final Pose pose15 = new Pose(50.429, 60.036);
    private static final Pose pose16 = new Pose(70.441, 21.100);
    private static final Pose pose17 = new Pose(70.441, 21.100);
    private static final Pose pose18 = new Pose(38.502, 83.529);
    private static final Pose pose19 = new Pose(38.502, 83.529);
    private static final Pose pose20 = new Pose(15.227, 83.964);
    private static final Pose pose21 = new Pose(15.227, 83.964);
    private static final Pose pose22 = new Pose(59.782, 83.529);
    private static final Pose pose23 = new Pose(59.782, 83.529);
    private static final Pose pose24 = new Pose(65.873, 77.873);
    private TelemetryManager panelsTelemetry; // Panels Telemetry instance
    public Follower follower; // Pedro Pathing follower instance
    private int pathState; // Current autonomous path state (state machine)
    private Paths paths; // Paths defined in the Paths class

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 8, Math.toRadians(90)));

        paths = new Paths(follower); // Build paths

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void loop() {
        follower.update(); // Update Pedro Pathing
        pathState = autonomousPathUpdate(); // Update autonomous state machine

        // Log values to Panels and Driver Station
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public static class Paths {

        public PathChain Path1;
        public PathChain Path2;
        public PathChain Path3;
        public PathChain Path4;
        public PathChain Path5;
        public PathChain Path6;
        public PathChain Path7;
        public PathChain Path8;
        public PathChain Path9;
        public PathChain Path10;
        public PathChain Path11;
        public PathChain Path12;

        public Paths(Follower follower) {
            Path1 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose1, pose2)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(90), Math.toRadians(180))
                    .build();

            Path2 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose3, pose4)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path3 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose5, pose6)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(110))
                    .setReversed()
                    .build();

            Path4 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose7, pose8)
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path5 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose9, pose10)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(110), Math.toRadians(180))
                    .build();

            Path6 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose11, pose12)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path7 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose13, pose14)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(118))
                    .setReversed()
                    .build();

            Path8 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose15, pose16)
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();

            Path9 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose17, pose18)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(118), Math.toRadians(180))
                    .build();

            Path10 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose19, pose20)
                    )
                    .setTangentHeadingInterpolation()
                    .build();

            Path11 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose21, pose22)
                    )
                    .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(137))
                    .setReversed()
                    .build();

            Path12 = follower
                    .pathBuilder()
                    .addPath(
                            new BezierLine(pose23, pose24)
                    )
                    .setTangentHeadingInterpolation()
                    .setReversed()
                    .build();
        }
    }

    public int autonomousPathUpdate() {
        // Add your state machine Here
        // Access paths with paths.pathName
        // Refer to the Pedro Pathing Docs (Auto Example) for an example state machine
        return pathState;
    }
}

