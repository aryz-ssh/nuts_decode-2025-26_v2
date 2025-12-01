package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SorterLogicV2_indexing_experiment;

@TeleOp(name = "SorterIndexingTest", group = "Sorter")
public class SorterIndexingTest extends LinearOpMode {

    private SorterLogicV2_indexing_experiment sorter;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setMsTransmissionInterval(50);

        sorter = new SorterLogicV2_indexing_experiment();
        sorter.init(hardwareMap, telemetry);

        telemetry.addLine("Sorter initialized.");
        telemetry.addLine("Homing is mandatory.");
        telemetry.update();

        // ===============================
        //  INIT LOOP (HOMING + TUNING)
        // ===============================
        while (opModeInInit()) {

            sorter.init_loop();  // runs homing, tuning, and update()

            telemetry.addLine("INIT LOOP ACTIVE");
            telemetry.addData("Homed", sorter.sorterHomed);
            telemetry.addData("Tuned", sorter.tuned);
            telemetry.addData("Alpha", sorter.getAlpha());
            telemetry.update();
        }

        // ===============================
        //  AUTO INDEXING BEGINS
        // ===============================

        waitForStart();

        if (isStopRequested()) return;

        sorter.startAutoIntake();
        telemetry.addLine("Auto intake started.");
        telemetry.update();

        // ===============================
        //  RUNTIME LOOP
        // ===============================

        while (opModeIsActive()) {

            sorter.update();

            telemetry.addLine("---- SORTER STATUS ----");
            telemetry.addData("Alpha", sorter.getAlpha());
            telemetry.addData("R", sorter.getRed());
            telemetry.addData("G", sorter.getGreen());
            telemetry.addData("B", sorter.getBlue());
            telemetry.addLine();

            telemetry.addData("Ball Present", sorter.isBallPresent());
            telemetry.addData("Detected Color", sorter.detectBallColor());
            telemetry.addLine();

            telemetry.addData("Encoder", sorter.getCurrentPos());
            telemetry.addData("Target", sorter.getTargetPos());
            telemetry.update();
        }
    }
}
