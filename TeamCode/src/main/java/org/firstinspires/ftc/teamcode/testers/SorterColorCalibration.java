package org.firstinspires.ftc.teamcode.testers;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp(name = "Sorter Color Calibration", group = "Test")
@Config
public class SorterColorCalibration extends LinearOpMode {

    // ---------------- HARDWARE ----------------
    private ColorSensor sorterColorSensor;

    // ---------------- TUNABLES ----------------
    public static float COLOR_DOMINANCE_RATIO = 1.15f;
    public static float COLOR_CONFIDENCE_MIN  = 0.05f;
    public static int REQUIRED_STABLE_FRAMES  = 3;

    // ---------------- ENUM ----------------
    public enum BallColor {
        PURPLE,
        GREEN,
        UNKNOWN
    }

    // ---------------- STATE ----------------
    private BallColor lastCandidate = BallColor.UNKNOWN;
    private int stableCount = 0;

    @Override
    public void runOpMode() {

        sorterColorSensor = hardwareMap.get(ColorSensor.class, "sorterColorSensor");

        telemetry.addLine("Sorter Color Calibration Ready");
        telemetry.addLine("Hole-based sensing enabled");
        telemetry.addLine("Tune RATIO / CONFIDENCE / STABLE_FRAMES");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {

            // ---------- RAW ----------
            int alphaRaw = sorterColorSensor.alpha();
            int rRaw = sorterColorSensor.red();
            int gRaw = sorterColorSensor.green();
            int bRaw = sorterColorSensor.blue();

            // ---------- NORMALIZE ----------
            float a = alphaRaw / 1000f;
            float r = rRaw / 1000f;
            float g = gRaw / 1000f;
            float b = bRaw / 1000f;

            // ---------- DIFFS ----------
            float diffRG = Math.abs(r - g);
            float diffGB = Math.abs(g - b);
            float diffBR = Math.abs(b - r);

            // ---------- VOID CHECK ----------
            boolean hardVoid =
                    a < 0.05f &&
                            diffRG < 0.01f &&
                            diffGB < 0.01f &&
                            diffBR < 0.01f;

            // ---------- DOMINANCE ----------
            boolean strongPurple =
                    b > g * COLOR_DOMINANCE_RATIO &&
                            b > r * COLOR_DOMINANCE_RATIO &&
                            (b - Math.max(g, r)) > COLOR_CONFIDENCE_MIN;

            boolean weakPurple =
                    b > g * COLOR_DOMINANCE_RATIO &&
                            b > r * COLOR_DOMINANCE_RATIO;

            boolean strongGreen =
                    g > b * COLOR_DOMINANCE_RATIO &&
                            g > r * COLOR_DOMINANCE_RATIO &&
                            (g - Math.max(b, r)) > COLOR_CONFIDENCE_MIN;

            boolean weakGreen =
                    g > b * COLOR_DOMINANCE_RATIO &&
                            g > r * COLOR_DOMINANCE_RATIO;

            // ---------- CANDIDATE ----------
            BallColor candidate = BallColor.UNKNOWN;

            if (!hardVoid) {
                if (strongPurple) candidate = BallColor.PURPLE;
                else if (strongGreen) candidate = BallColor.GREEN;
                else if (lastCandidate == BallColor.PURPLE && weakPurple)
                    candidate = BallColor.PURPLE;
                else if (lastCandidate == BallColor.GREEN && weakGreen)
                    candidate = BallColor.GREEN;
            }

            // ---------- TEMPORAL STABILITY ----------
            if (candidate == lastCandidate && candidate != BallColor.UNKNOWN) {
                stableCount++;
            } else {
                stableCount = 0;
                lastCandidate = candidate;
            }

            BallColor detected =
                    (stableCount >= REQUIRED_STABLE_FRAMES)
                            ? candidate
                            : BallColor.UNKNOWN;

            // ---------- TELEMETRY ----------
            telemetry.addLine("=== RAW ===");
            telemetry.addData("Alpha", alphaRaw);
            telemetry.addData("R", rRaw);
            telemetry.addData("G", gRaw);
            telemetry.addData("B", bRaw);

            telemetry.addLine("=== RATIOS ===");
            telemetry.addData("B/G", g > 0 ? b / g : 0);
            telemetry.addData("B/R", r > 0 ? b / r : 0);
            telemetry.addData("G/B", b > 0 ? g / b : 0);
            telemetry.addData("G/R", r > 0 ? g / r : 0);

            telemetry.addLine("=== STATE ===");
            telemetry.addData("Hard Void", hardVoid);
            telemetry.addData("Candidate", candidate);
            telemetry.addData("Stable Count", stableCount);
            telemetry.addData("Detected", detected);

            telemetry.update();
        }
    }
}
