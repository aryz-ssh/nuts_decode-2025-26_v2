package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.FinalSorter;

@TeleOp(name = "TUNE - Sorter Color Sensor", group = "Tuning")
public class SorterColorTuning extends LinearOpMode {

    private FinalSorter sorter;

    @Override
    public void runOpMode() {

        sorter = new FinalSorter();
        sorter.init(hardwareMap, telemetry);

        telemetry.addLine("Sorter Color Tuning READY");
        telemetry.addLine("Sorter locked at INTAKE");
        telemetry.addLine("Watch LED + Dashboard");
        telemetry.update();

        waitForStart();

        // Lock sorter at intake pocket 0
//        sorter.movePocketToIntake(0);
 
        while (opModeIsActive()) {

            // sorter.update();

            int r = sorter.getColorR();
            int g = sorter.getColorG();
            int b = sorter.getColorB();
            int sum = r + g + b;

            int max = Math.max(r, Math.max(g, b));
            int min = Math.min(r, Math.min(g, b));
            int chroma = max - min;

            double nr = sum > 0 ? (double) r / sum : 0;
            double ng = sum > 0 ? (double) g / sum : 0;
            double nb = sum > 0 ? (double) b / sum : 0;

            double gOverR = r > 0 ? (double) g / r : 0;
            double gOverB = b > 0 ? (double) g / b : 0;
            double rOverG = g > 0 ? (double) r / g : 0;
            double rOverB = b > 0 ? (double) r / b : 0;

            FinalSorter.BallColor detected = sorter.getDetectedColorAtIntake();

            // -------- DRIVER STATION --------
            telemetry.addLine("---- RAW RGB ----");
            telemetry.addData("R", r);
            telemetry.addData("G", g);
            telemetry.addData("B", b);
            telemetry.addData("Sum", sum);

            telemetry.addLine("---- NORMALIZED ----");
            telemetry.addData("R%", "%.2f", nr);
            telemetry.addData("G%", "%.2f", ng);
            telemetry.addData("B%", "%.2f", nb);

            telemetry.addLine("---- RATIOS ----");
            telemetry.addData("G/R", "%.2f", gOverR);
            telemetry.addData("G/B", "%.2f", gOverB);
            telemetry.addData("R/G", "%.2f", rOverG);
            telemetry.addData("R/B", "%.2f", rOverB);

            telemetry.addLine("---- RESULT ----");
            telemetry.addData("Detected", detected);
            telemetry.addData("Chroma", chroma);
            telemetry.update();

            // -------- DASHBOARD --------
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("r", r);
            packet.put("g", g);
            packet.put("b", b);
            packet.put("sum", sum);

            packet.put("nr", nr);
            packet.put("ng", ng);
            packet.put("nb", nb);

            packet.put("g_over_r", gOverR);
            packet.put("g_over_b", gOverB);
            packet.put("r_over_g", rOverG);
            packet.put("r_over_b", rOverB);

            packet.put("detected", detected.toString());
            packet.put("chroma", chroma);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);

            sleep(20);
        }
    }
}
