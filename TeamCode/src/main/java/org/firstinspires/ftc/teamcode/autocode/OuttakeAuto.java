package org.firstinspires.ftc.teamcode.autocode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Mechanisms;

public class OuttakeAuto {
    Mechanisms newMechanisms = new Mechanisms();

    public void initOuttakeAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        newMechanisms.initMechanisms(hardwareMap, telemetry, true);
    }

    private enum OuttakeAutoState {
        IDLE,
        CLOSE,
        FAR
    }

    private OuttakeAutoState OuttakeAutoState;
    private double closeAngle = 0.5;
    private double FarAngle = 0;





}
