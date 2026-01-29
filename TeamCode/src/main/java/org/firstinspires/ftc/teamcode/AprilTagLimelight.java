package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

@Config
public class AprilTagLimelight {

    private Limelight3A limelight;
    private IMU imu;

    // ================= TUNING =================
    public static double STRAFE_KP = 1.4;   // meters → strafe power
    public static double TURN_KP   = 0.02;  // degrees → turn power

    public static double MAX_STRAFE = 0.6;
    public static double MAX_TURN   = 0.5;

    public static double LATERAL_DEADBAND_M = 0.02; // 2 cm
    public static double HEADING_DEADBAND_D = 1.0;  // 1 deg

    private static final int AUTO_ALIGN_PIPELINE = 3;
    private static final int MOTIF_PIPELINE = 0;

    // ================= INIT =================
    public AprilTagLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(9); // AprilTag pipeline

        imu = hw.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        ));

        limelight.start();
    }

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

    // ================= INTERNAL HELPERS =================
    private LLResult getValidResult() {
        LLResult r = limelight.getLatestResult();
        return (r != null && r.isValid()) ? r : null;
    }

    private static double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }

    // ================= POSE =================
    public Pose3D getBotPoseMT2() {
        LLResult r = getValidResult();
        if (r == null) return null;
        return r.getBotpose_MT2();
    }

    // ================= ERROR TERMS =================
    public Double getLateralErrorMeters() {
        Pose3D pose = getBotPoseMT2();
        if (pose == null) return null;
        return pose.getPosition().y;
    }

    public Double getForwardDistanceMeters() {
        Pose3D pose = getBotPoseMT2();
        if (pose == null) return null;
        return pose.getPosition().x;
    }

    public Double getHeadingErrorDeg() {
        Pose3D pose = getBotPoseMT2();
        if (pose == null) return null;

        double yaw = pose.getOrientation().getYaw(AngleUnit.DEGREES);

        // Normalize to [-180, 180]
        while (yaw > 180) yaw -= 360;
        while (yaw < -180) yaw += 360;

        return yaw;
    }

    // ================= DRIVE CORRECTIONS =================

    /**
     * Returns strafe correction (robot-centric X)
     */
    public double getStrafeCorrection() {
        if (!isAutoAlignPipeline()) return 0.0;

        Double lateral = getLateralErrorMeters();
        if (lateral == null) return 0.0;

        if (Math.abs(lateral) < LATERAL_DEADBAND_M) return 0.0;

        double cmd = -lateral * STRAFE_KP;
        return clamp(cmd, MAX_STRAFE);
    }

    /**
     * Returns turn correction ONLY when heading lock is enabled
     */
    public double getTurnCorrection(boolean enabled) {
        if (!enabled) return 0.0;

        LLResult r = limelight.getLatestResult();
        if (r == null || !r.isValid()) return 0.0;

        double tx = r.getTx();
        if (Math.abs(tx) < 1.0) return 0.0;

        return clamp(tx * TURN_KP, MAX_TURN);
    }

    public void enableAutoAlign() {
        limelight.pipelineSwitch(AUTO_ALIGN_PIPELINE);
    }

    public boolean isAutoAlignPipeline() {
        LLResult r = limelight.getLatestResult();
        return r != null && r.getPipelineIndex() == AUTO_ALIGN_PIPELINE;
    }


    // ================= MOTIF =================
    public void enableMotifScan() {
        limelight.pipelineSwitch(MOTIF_PIPELINE);
    }

    public Integer getMotifNumber() {
        LLResult r = getValidResult();
        if (r == null) return null;

        if (r.getFiducialResults().isEmpty()) return null;

        for (LLResultTypes.FiducialResult fid : r.getFiducialResults()) {
            int id = fid.getFiducialId();
            if (id == 21 || id == 22 || id == 23) {
                return id;
            }
        }

        return null; // no motif tag visible
    }

    public String getMotif() {
        Integer id = getMotifNumber();
        if (id == null) return "UNKNOWN";

        switch (id) {
            case 21: return "GPP";
            case 22: return "PGP";
            case 23: return "PPG";
            default: return "UNKNOWN";
        }
    }

    // ================= PIPELINE =================
    public void setPipeline(int pipeline) {
        limelight.pipelineSwitch(pipeline);
    }

    public int getCurrentPipeline() {
        LLResult r = limelight.getLatestResult();
        return (r != null) ? r.getPipelineIndex() : -1;
    }

    public boolean isConnected() {
        return limelight != null;
    }
}
