package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class AprilTagMotifScanner {


    private Limelight3A limelight;
    private IMU imu;

    private void initLimelight(HardwareMap hw) {
        limelight = hw.get(Limelight3A.class, "limelight");
        limelight.pipelidxrstch(2); //april tag pipline black n white
        imu = hw.get(IMU.class, "imu");
        RevHubOrientationOnRobot revHubOrientationOnRobot = new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(revHubOrientationOnRobot));

        limelight.start();
    }

    private void motifScanner() {
        List<BarcodeResult> barcodes = result.getBarcodeResults();
        for (BarcodeResult barcode : barcodes) {
            String data = barcode.getData(); // What the barcode says
            String family = barcode.getFamily(); // What type of barcode it is
            telemetry.addData("Barcode", data + " (" + family + ")");
        }
    }


}
