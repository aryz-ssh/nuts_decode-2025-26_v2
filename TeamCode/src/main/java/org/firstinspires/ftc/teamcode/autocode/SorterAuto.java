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
import org.firstinspires.ftc.teamcode.PostNut;
import org.firstinspires.ftc.teamcode.SorterLogicColor;

public class SorterAuto {
    private ElapsedTime stateTimer = new ElapsedTime();
    Mechanisms newMechanisms = new Mechanisms();
    PostNut newPostNut = new PostNut();


    private enum SorterAutoState {
        IDLE,
        POS1,
        POS2,
        POS3
    }

    private SorterAutoState sorterAutoState;

    public void initOuttakeAuto(HardwareMap hardwareMap, Telemetry telemetry) {
        newMechanisms.initMechanisms(hardwareMap, telemetry);
        sorterAutoState = SorterAutoState.IDLE;
       // newMechanisms.sorterLogic
    }

/*
* PSEUDOCODE FOR SORTER:
* PRELOAD: Preload all 3 balls in order GPP (Green is top, purple is other 2 slots
* For motif GPP: just start with top slot and rotate right, shooting each slot
* For motif PGP: just rotate to bottom left slot (purple2), then keep rotating right and shoot
* For motif PPG: just rotate to bottom right slot (purple1), then keep rotating right and shoot
* */







}