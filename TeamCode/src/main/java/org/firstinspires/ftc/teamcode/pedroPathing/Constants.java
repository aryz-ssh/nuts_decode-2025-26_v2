package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.ThreeWheelIMUConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.MecanumDrive;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.1538)
            .forwardZeroPowerAcceleration(-27.976707401431)
            .lateralZeroPowerAcceleration(-58.838768513832)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.006, 0.025))
            .headingPIDFCoefficients(new PIDFCoefficients(0.7, 0, 0.04, 0.02));
            // .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.03, 0,0.0001,0.3,0.025));
            // .centripetalScaling(0.0005);

    public static PathConstraints pathConstraints = new PathConstraints(0.99,
            100,
            0.75,
            1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(0.7)
            .rightFrontMotorName("rightFront")
            .rightRearMotorName("rightBack")
            .leftRearMotorName("leftBack")
            .leftFrontMotorName("leftFront")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(63.931380392050265)
            .yVelocity(42.99333541698602);

    public static ThreeWheelIMUConstants localizerConstants = new ThreeWheelIMUConstants()
            .forwardTicksToInches(0.0060564408396097)
            .strafeTicksToInches(0.0029499244889298)
            .turnTicksToInches(0.0019681621148874)
            .leftPodY(7)
            .rightPodY(-7)
            .strafePodX(-1.5)
            .leftEncoder_HardwareMapName("rightBack")
            .rightEncoder_HardwareMapName("leftBack")
            .strafeEncoder_HardwareMapName("leftFront")
            .leftEncoderDirection(Encoder.FORWARD)
            .rightEncoderDirection(Encoder.REVERSE)
            .strafeEncoderDirection(Encoder.REVERSE)
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.RIGHT, RevHubOrientationOnRobot.UsbFacingDirection.UP));

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .threeWheelIMULocalizer(localizerConstants)
                .build();
    }
}




