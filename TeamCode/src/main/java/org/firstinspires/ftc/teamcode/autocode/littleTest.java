package org.firstinspires.ftc.teamcode.autocode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous(name = "ForwardRightPedro", group = "Autonomous")
public class littleTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        Follower follower = Constants.createFollower(hardwareMap);

        // Start position
        Pose start = new Pose(0, 0, Math.toRadians(0));
        follower.setStartingPose(start);

        // ---------------- PATH BUILDING ----------------

        // 1) Forward 56 inches
        Pose forward56 = new Pose(56, 0, Math.toRadians(0));

        // 2) Turn right 90 degrees (same XY, new heading)
        Pose turnRight = new Pose(56, 0, Math.toRadians(-90));

        // 3) Forward 20 inches after the turn
        Pose forward20 = new Pose(56, 20, Math.toRadians(-90));

        PathChain path = follower.pathBuilder()
                // forward 56
                .addPath(new BezierLine(start, forward56))

                // rotate in place
                .addPath(new BezierLine(forward56, turnRight))

                // forward 20
                .addPath(new BezierLine(turnRight, forward20))

                .setTangentHeadingInterpolation()
                .build();

        waitForStart();

        follower.followPath(path);

        // MAIN LOOP
        while (opModeIsActive()) {
            follower.update();
        }
    }
}
