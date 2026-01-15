package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class AprilTagLimelight {

    private Limelight3A limelight;

    // ---- TUNING CONSTANTS ----
    private static final double STRAFE_KP = 0.035;
    private static final double STRAFE_DEADBAND = 0.7;
    private static final double MAX_STRAFE_POWER = 0.6;

    // Constructor
    public AprilTagLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // AprilTag pipeline
    }

    // ---------------- BASIC METHODS ----------------


    public double getTx() {
        return limelight.getLatestResult().getTx();

    }

    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    public double getDistance(){
        double targetOffsetAngle_Vertical = getTy();

        // how many degrees back is your limelight rotated from perfectly vertical?
        double limelightMountAngleDegrees = 10.0;

        // distance from the center of the Limelight lens to the floor
        double limelightLensHeightInches = 16.1;

        // distance from the target to the floor
        double goalHeightInches = 25.0;

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

        //calculate distance
        double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
        return distanceFromLimelightToGoalInches;

    }

    // ---------------- AUTO STRAFE ----------------

    /**
     * Returns strafe power to align robot with AprilTag
     * Positive = strafe right
     * Negative = strafe left
     */
    public double getAutoStrafePower(boolean enable) {

        double tx = getTx();

        // Deadband
        if (Math.abs(tx) < STRAFE_DEADBAND) return 0.0;

        double power = tx * STRAFE_KP;

        // Clamp power
        power = Math.max(-MAX_STRAFE_POWER, Math.min(MAX_STRAFE_POWER, power));

        return power;
    }

    // ---------------- OPTIONAL TURN ALIGN ----------------

    public double getAutoAlignTurn(boolean enable, double driverTurn) {

        double kP = 0.02;
        double deadband = 1.0;
        double tx = getTx();

        if (Math.abs(tx) < deadband) return 0.0;
        return tx * kP;
    }

    // ---------------- PIPELINE ----------------

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public int getCurrentPipeline() {
        LLResult result = limelight.getLatestResult();
        return (result != null) ? result.getPipelineIndex() : -1;
    }

    public boolean isConnected() {
        return limelight != null;
    }
}
