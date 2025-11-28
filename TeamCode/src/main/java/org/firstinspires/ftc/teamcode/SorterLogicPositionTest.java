package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Sorter Logic Position Test", group="Testing")
public class SorterLogicPositionTest extends LinearOpMode {

    SorterLogic sorter;

    // State used for cycling pockets
    int pocket = 1;   // 1 → 2 → 3
    boolean lastDpadUp = false;
    boolean lastDpadDown = false;

    @Override
    public void runOpMode() {

        sorter = new SorterLogic();
        sorter.init(hardwareMap, telemetry);

        // -----------------------------
        // INIT LOOP: perform homing
        // -----------------------------
        while (!isStarted() && !isStopRequested()) {
            sorter.init_loop();
        }

        // If stop pressed during init
        if (isStopRequested()) return;

        telemetry.clearAll();
        telemetry.addLine("Sorter ready!");
        telemetry.update();

        // -----------------------------
        // RUN LOOP
        // -----------------------------
        while (opModeIsActive()) {

            // --- Cycle pockets with Dpad ---
            boolean dpadUp   = gamepad1.dpad_up;
            boolean dpadDown = gamepad1.dpad_down;

            // Cycle pocket +1
            if (dpadUp && !lastDpadUp) {
                pocket++;
                if (pocket > 3) pocket = 1;
            }

            // Cycle pocket -1
            if (dpadDown && !lastDpadDown) {
                pocket--;
                if (pocket < 1) pocket = 3;
            }

            lastDpadUp = dpadUp;
            lastDpadDown = dpadDown;

            // --- Move to INTAKE position (A) ---
            if (gamepad1.a) {
                sorter.goToIntake(pocket);
            }

            // --- Move to OUTTAKE position (B) ---
            if (gamepad1.b) {
                sorter.goToOuttake(pocket);
            }

            // --- Stop (X) ---
            if (gamepad1.x) {
                sorter.goToPosition(sorter.B1_INTAKE); // default home intake
            }

//            // --- Manual calibration tweak (RB/LB) ---
//            if (gamepad1.right_bumper) {
//                sorter.goToPosition(sorterMotor.getCurrentPosition() + 5);
//            }
//            if (gamepad1.left_bumper) {
//                sorter.goToPosition(sorterMotor.getCurrentPosition() - 5);
//            }

            // --- Update PID and control ---
            sorter.update();

            // TELEMETRY
            telemetry.addData("Pocket", pocket);
            telemetry.addData("Homed", sorter.isHomed());
            telemetry.addData("Target", sorter.targetPos);
            telemetry.addData("Position", sorter.sorterMotor.getCurrentPosition());
            telemetry.addData("Moving", sorter.moving);
            telemetry.update();
        }
    }
}
