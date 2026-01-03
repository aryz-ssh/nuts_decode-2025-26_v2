package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import org.firstinspires.ftc.teamcode.Mechanisms;

@Config

public class PostNut_LimelightController {


    // ---- Tuning ----
    public static double KP = 0.02;
    public static double KD = 0.001;
    public static double MAX_TURN = 0.6;
    public static double TX_DEADBAND = 0.75;
    public static double TX_TRIM = 0.0;

    private boolean aimLockEnabled = false;
    private boolean lastToggle = false;

    private double lastTx = 0;
    private double lastError = 0;
    private int noTargetFrames = 0;
    private static final int MAX_NO_TARGET_FRAMES = 3;

    // --- Telemetry state ---
    private boolean llConnected = false;
    private boolean llValid = false;
    private boolean hasTarget = false;
    private double tx = 0;
    private int fiducialCount = 0;
    private int lockedFiducial = -1;
    private int currentPipeline = -1;

    private final Mechanisms mechanisms;

    public PostNut_LimelightController(Mechanisms mechanisms) {
        this.mechanisms = mechanisms;
    }

    /** Toggle aim lock */
    public void updateToggle(boolean toggleButton) {
        if (toggleButton && !lastToggle) {
            aimLockEnabled = !aimLockEnabled;
        }
        lastToggle = toggleButton;
    }

    public void setPipeline(int pipeline) {
        mechanisms.limelight.pipelineSwitch(pipeline);
        mechanisms.limelight.start();
        currentPipeline = pipeline;
    }

    /** Returns corrected turn command */
    public double getTurnCorrection(double manualTurn) {

        if (!aimLockEnabled) {
            lastError = 0;
            return manualTurn;
        }

        LLResult result = mechanisms.limelight.getLatestResult();
        boolean hasTarget = false;
        double tx = 0;

        if (result != null && result.isValid()
                && !result.getFiducialResults().isEmpty()) {

            hasTarget = true;
            tx = result.getTx();
            lastTx = tx;
            noTargetFrames = 0;

        } else {
            noTargetFrames++;
            if (noTargetFrames <= MAX_NO_TARGET_FRAMES) {
                hasTarget = true;
                tx = lastTx;
            }
        }

        if (!hasTarget) {
            lastError = 0;
            return manualTurn;
        }

        double correctedTx = tx + TX_TRIM;
        double error = Math.abs(correctedTx) < TX_DEADBAND ? 0 : correctedTx;
        double derivative = error - lastError;

        double turn = (KP * error) + (KD * derivative);
        turn = Math.max(-MAX_TURN, Math.min(MAX_TURN, turn));

        lastError = error;
        return turn;
    }

    public boolean isAimLockEnabled() {
        return aimLockEnabled;
    }

    public void update() {

        if (mechanisms.limelight == null) {
            llConnected = false;
            llValid = false;
            hasTarget = false;
            tx = 0;
            fiducialCount = 0;
            lockedFiducial = -1;
            return;
        }

        LLResult result = mechanisms.limelight.getLatestResult();

        llConnected = (result != null);
        llValid = (result != null && result.isValid());

        hasTarget = false;
        fiducialCount = 0;
        lockedFiducial = -1;

        if (llValid && !result.getFiducialResults().isEmpty()) {
            hasTarget = true;
            tx = result.getTx();
            lastTx = tx;
            fiducialCount = result.getFiducialResults().size();
            lockedFiducial = result.getFiducialResults().get(0).getFiducialId();
            noTargetFrames = 0;
        } else {
            noTargetFrames++;
            if (noTargetFrames <= MAX_NO_TARGET_FRAMES) {
                hasTarget = true;
                tx = lastTx;
            } else {
                tx = 0;
            }
        }
    }

    public boolean isConnected() {
        return llConnected;
    }

    public boolean isValid() {
        return llValid;
    }

    public boolean hasTarget() {
        return hasTarget;
    }

    public double getTx() {
        return tx;
    }

    public int getFiducialCount() {
        return fiducialCount;
    }

    public int getLockedFiducial() {
        return lockedFiducial;
    }

    public int getCurrentPipeline() {
        return currentPipeline;
    }
}
