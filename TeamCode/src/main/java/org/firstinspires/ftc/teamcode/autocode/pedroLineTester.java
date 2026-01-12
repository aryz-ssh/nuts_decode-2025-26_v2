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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.CustomMecanumPedroDrivebase;

@Autonomous(name = "Pedro Back And Forth DEBUG", group = "Pedro Debug")
@Configurable
public class pedroLineTester extends LinearOpMode {

    private TelemetryManager panelsTelemetry;
    private Follower follower;

    // ================= TUNABLES =================
    public static double AX = 88.0;
    public static double AY = 8.0;

    public static double BX = 88.0;
    public static double BY = 35.0;

    // Degrees for readability → converted to radians
    public static double START_HEADING_DEG = 90.0;

    // HARD ROTATION CLAMP (NON-NEGOTIABLE)
    public static double MAX_TURN = 0.15;

    // ================= PATHS =================
    private PathChain pathAB;
    private PathChain pathBA;

    private boolean goingAB = true;
    private int trips = 0;

    @Override
    public void runOpMode() {

        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

        follower = Constants.createFollower(hardwareMap);

        Pose A = new Pose(AX, AY, Math.toRadians(START_HEADING_DEG));
        Pose B = new Pose(BX, BY, Math.toRadians(START_HEADING_DEG));

        // EXPLICIT POSE INITIALIZATION (DO BOTH)
        follower.setStartingPose(A);
        follower.setPose(A);

        // ================= BUILD PATHS =================
        pathAB = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(AX, AY),
                        new Pose(BX, BY)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(START_HEADING_DEG))
                .build();

        pathBA = follower.pathBuilder()
                .addPath(new BezierLine(
                        new Pose(BX, BY),
                        new Pose(AX, AY)
                ))
                .setConstantHeadingInterpolation(Math.toRadians(START_HEADING_DEG))
                .build();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);

        waitForStart();

        // Start A → B
        follower.followPath(pathAB);

        while (opModeIsActive()) {

            follower.update();

            // Flip direction when finished
            if (!follower.isBusy()) {
                goingAB = !goingAB;
                trips++;

                if (goingAB) {
                    follower.followPath(pathAB);
                } else {
                    follower.followPath(pathBA);
                }
            }

            // ================= TELEMETRY =================
            Pose p = follower.getPose();

            panelsTelemetry.debug("Dir", goingAB ? "A → B" : "B → A");
            panelsTelemetry.debug("Trips", trips);
            panelsTelemetry.debug("X", String.format("%.2f", p.getX()));
            panelsTelemetry.debug("Y", String.format("%.2f", p.getY()));
            panelsTelemetry.debug("Heading(rad)", String.format("%.3f", p.getHeading()));
            panelsTelemetry.update(telemetry);
        }
    }
}
