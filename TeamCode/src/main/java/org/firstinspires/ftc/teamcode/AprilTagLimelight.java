package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class AprilTagLimelight {

    private Limelight3A limelight;

    // Constructor
    public AprilTagLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(8); // default pipeline
    }

    // ---------------- BASIC METHODS ----------------

    // Returns true if Limelight sees a target

    public int hasTarget() {
        LLResult result = limelight.getLatestResult();
        return (result != null && result.isValid()) ? 1 : 0;
    }

    // Returns true if the Limelight sees a target


    // Returns horizontal offset from target (tx)
    public double getTx() {
        return limelight.getLatestResult().getTx();
    }

    // Returns vertical offset from target (ty)
    public double getTy() {
        return limelight.getLatestResult().getTy();
    }

    // Optional: distance calculation example
    public double getDistance() {
        double targetHeight = 24.0; // inches
        double cameraHeight = 8.0;  // inches
        double cameraAngle = 20.0;  // degrees
        double ty = getTy();
        double radians = Math.toRadians(cameraAngle + ty);
        return (targetHeight - cameraHeight) / Math.tan(radians);
    }

    // ---------------- PIPELINE ----------------

    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public int getCurrentPipeline() {
        return limelight.getLatestResult().getPipelineIndex();
    }

    // ---------------- AUTO-ALIGN ----------------

    public double getAutoAlignTurn(boolean forceAlign, double currentRx) {
        boolean onTarget = hasTarget() == 1; // declare here
        if (forceAlign && onTarget) {
            double kP = 0.02;
            double tx = getTx();
            double deadband = 1.0;
            if (Math.abs(tx) < deadband) return 0;
            return tx * kP;
        }
        return currentRx;
    }

    // Optional: simple update loop if needed
    public void update() {
        // Could add smoothing, LED control, etc.
    }

    // Add getters if you need for telemetry

    public int getFiducialCount() {
        return limelight.getLatestResult().getTa() > 0 ? 1 : 0; // simplified
    }

    public double getCurrentTurnCmd() {
        return 0; // placeholder if you want to store auto-turn output
    }

    public boolean isConnected() {
        return limelight != null;
    }

    public boolean isValid() {
        return true; // placeholder
    }

    public boolean isAimLockEnabled() {
        return false; // placeholder, implement if you add toggle logic
    }

    public int getLockedFiducial() {
        return -1; // placeholder, implement if you store locked tag ID
    }
}
