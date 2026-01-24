package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotorEx;

public class FinalSorter {

    public enum BallColor { NONE, GREEN, PURPLE }

    private class Slot {
        BallColor color = BallColor.NONE;
        double angle;
    }

    private Slot[] slots;
    private DcMotorEx sorterMotor;

    private static final double OUTTAKE_ANGLE = 180;
    private int ticksPer60;

    private BallColor pendingBall = BallColor.NONE;
    private int ballCount = 0;

    // -----------------------------
    // Constructor
    // -----------------------------
    public void SorterLogic(DcMotorEx motor, int ticksPer60) {
        this.sorterMotor = motor;
        this.ticksPer60 = ticksPer60;

        // Initialize slots
        slots = new Slot[3];
        slots[0] = new Slot(); slots[0].angle = 0;
        slots[1] = new Slot(); slots[1].angle = 120;
        slots[2] = new Slot(); slots[2].angle = 240;

        // Reset motor
        sorterMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        sorterMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        sorterMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    // -----------------------------
    // Public interface
    // -----------------------------

    // Rotate 120° for intake
    public void rotateIntakeSlot() {
        rotateWheelTicks(ticksPer60 * 2);
        rotateModel(120);

        if (pendingBall != BallColor.NONE && ballCount < 3) {
            slots[0].color = pendingBall; // bottom slot is intake
            pendingBall = BallColor.NONE;
            ballCount++;
        }
    }

    // Rotate closest ball of given color to top/outtake
    public void rotateClosestToTop(BallColor color) {
        Slot s = findClosest(color);
        if (s == null) return;

        double delta = OUTTAKE_ANGLE - s.angle;
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        int ticks = (int)(delta / 60.0 * ticksPer60);
        rotateWheelTicks(ticks);
        rotateModel(delta);
    }

    // Store ball when detected by color sensor
    public void markPendingBall(BallColor color) {
        pendingBall = color;
    }

    public BallColor getSlotColor(int index) {
        return slots[index].color;
    }

    public double getSlotAngle(int index) {
        return slots[index].angle;
    }

    // -----------------------------
    // Internal helpers
    // -----------------------------
    private void rotateWheelTicks(int ticks) {
        int target = sorterMotor.getCurrentPosition() + ticks;
        sorterMotor.setTargetPosition(target);
        sorterMotor.setPower(0.5);
        while (sorterMotor.isBusy()) { /* wait */ }
        sorterMotor.setPower(0);
    }

    private void rotateModel(double delta) {
        for (Slot s : slots) {
            s.angle = (s.angle + delta) % 360;
        }
    }

    private Slot findClosest(BallColor color) {
        Slot best = null;
        double bestDist = Double.MAX_VALUE;
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
}
