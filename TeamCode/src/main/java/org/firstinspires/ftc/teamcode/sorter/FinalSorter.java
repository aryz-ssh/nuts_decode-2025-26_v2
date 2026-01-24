package org.firstinspires.ftc.teamcode.sorter;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name="Full Sorter Logic", group="Sorter")
public class FinalSorter extends LinearOpMode {

    // Replace this with your measured ticks per 60 degrees
    private static final int TICKS_PER_60 = 240; // example, update after testing
    private static final int TICKS_PER_120 = TICKS_PER_60 * 2;

    private static final double OUTTAKE_ANGLE = 180;

    private DcMotorEx sorterMotor;

    private enum BallColor { NONE, GREEN, PURPLE }

    private class Slot {
        BallColor color = BallColor.NONE;
        double angle;
    }

    private Slot[] slots = new Slot[3];
    private BallColor pendingBall = BallColor.NONE;
    private int ballCount = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        sorterMotor = hardwareMap.get(DcMotorEx.class, "sorterMotor");

        // Initialize motor
        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        // Initialize slots
        slots[0] = new Slot(); slots[0].angle = 0;   // Slot 1 = bottom
        slots[1] = new Slot(); slots[1].angle = 120; // Slot 2
        slots[2] = new Slot(); slots[2].angle = 240; // Slot 3

        telemetry.addLine("Sorter Ready");
        telemetry.addLine("RB = intake rotate, X = GREEN to top, Y = PURPLE to top");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ----------------------------
            // Intake button: rotate 120° to next slot
            // ----------------------------
            if (gamepad1.right_bumper) {
                rotateWheelTicks(TICKS_PER_120);
                rotateModel(120);

                // commit pending ball
                if (pendingBall != BallColor.NONE && ballCount < 3) {
                    slots[0].color = pendingBall; // bottom slot is always intake
                    pendingBall = BallColor.NONE;
                    ballCount++;
                }

                sleep(300); // small debounce to avoid double rotation
            }

            // ----------------------------
            // Outtake buttons
            // ----------------------------
            if (gamepad1.x) {
                bringClosestToTop(BallColor.GREEN);
                sleep(300);
            }

            if (gamepad1.y) {
                bringClosestToTop(BallColor.PURPLE);
                sleep(300);
            }

            // ----------------------------
            // Telemetry
            // ----------------------------
            telemetry.addData("Slot1", slots[0].color + " @ " + slots[0].angle);
            telemetry.addData("Slot2", slots[1].color + " @ " + slots[1].angle);
            telemetry.addData("Slot3", slots[2].color + " @ " + slots[2].angle);
            telemetry.addData("Pending Ball", pendingBall);
            telemetry.addData("Encoder", sorterMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    // ----------------------------
    // Helper functions
    // ----------------------------

    private void rotateWheelTicks(int ticks) {
        int target = sorterMotor.getCurrentPosition() + ticks;
        sorterMotor.setTargetPosition(target);
        sorterMotor.setPower(0.5);

        while (sorterMotor.isBusy() && opModeIsActive()) { /* wait */ }

        sorterMotor.setPower(0);
    }

    private void rotateModel(double delta) {
        for (Slot s : slots) {
            s.angle = (s.angle + delta) % 360;
        }
    }

    private Slot findClosest(BallColor color) {
        Slot best = null;
        double bestDist = 999;
        for (Slot s : slots) {
            if (s.color == color) {
                double d = Math.abs(s.angle - OUTTAKE_ANGLE);
                d = Math.min(d, 360 - d);
                if (d < bestDist) {
                    bestDist = d;
                    best = s;
                }
            }
        }
        return best;
    }

    private void bringClosestToTop(BallColor color) {
        Slot s = findClosest(color);
        if (s == null) return;

        // calculate shortest rotation
        double delta = OUTTAKE_ANGLE - s.angle;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        // convert degrees to ticks
        int ticks = (int)(delta / 60.0 * TICKS_PER_60);
        rotateWheelTicks(ticks);
        rotateModel(delta);
    }

    // Simulated intake sensor (you would replace this with real color sensor reading)
    private void onBallDetected(BallColor detected) {
        pendingBall = detected;
    }
}
