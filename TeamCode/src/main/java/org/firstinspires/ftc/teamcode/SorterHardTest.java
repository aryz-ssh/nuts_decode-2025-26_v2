package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;


@TeleOp(name="Sorter HARD Test", group="Debug")
public class SorterHardTest extends LinearOpMode {

    DcMotorEx sorter;

    @Override
    public void runOpMode() {
        sorter = hardwareMap.get(DcMotorEx.class, "sorterMotor");

        sorter.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        sorter.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);

        waitForStart();

        while (opModeIsActive()) {
            sorter.setPower(0.4);
            telemetry.addData("Power", 0.4);
            telemetry.update();
        }
    }
}
