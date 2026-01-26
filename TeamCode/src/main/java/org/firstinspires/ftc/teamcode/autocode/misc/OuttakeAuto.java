package org.firstinspires.ftc.teamcode.autocode.misc;

import com.qualcomm.robotcore.hardware.HardwareMap;

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
