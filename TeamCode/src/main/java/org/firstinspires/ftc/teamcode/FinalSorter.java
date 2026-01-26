package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class FinalSorter {

    // ================= CONSTANTS =================
    public static double TICKS_PER_REV = 537;
    public static double SORTER_VELOCITY = 600;

    public static double SORTER_P = 16;
    public static double SORTER_I = 0.0;
    public static double SORTER_D = 0.02;
    public static double SORTER_F = 19;

    private static final int SLOT_COUNT = 3;
    private static final int TICKS_PER_120 =
            (int)Math.round(TICKS_PER_REV / 3.0);

    // ================= COLOR SENSOR TUNING =================
    public static double COLOR_MIN_SUM = 300;   // minimum RGB sum to consider "ball present"

    // Ratio thresholds (tune on Dashboard)
    public static double GREEN_G_OVER_R = 1.4;
    public static double GREEN_G_OVER_B = 1.4;

    public static double PURPLE_R_OVER_G = 1.3;
    public static double PURPLE_R_OVER_B = 1.3;

    // ================= TYPES =================
    public enum BallColor { NONE, GREEN, PURPLE }

    private static class Slot {
        BallColor color = BallColor.NONE;
    }

    // ================= HARDWARE =================
    private DcMotorEx motor;
    private DigitalChannel beamBreak;
    private ColorSensor colorSensor;
    private Telemetry telemetry;
    private Servo statusLEDServo;
    private StatusLED_RGB statusLED;

    // ================= MODEL =================
    private final Slot[] slots = new Slot[SLOT_COUNT];
    private int currentSlot = 0;          // physical slot at intake
    private int targetSlot = -1;

    // ================= STATE =================
    private boolean busy = false;
    private boolean lastBeamClear = true;
    private BallColor pendingBall = BallColor.NONE;
    private double wheelAngleDeg = 0.0; // 0° = intake-aligned at init

    // ================= INIT =================
    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = hw.get(DcMotorEx.class, "sorterMotor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        motor.setPIDFCoefficients(
                DcMotorEx.RunMode.RUN_USING_ENCODER,
                new PIDFCoefficients(
                        SORTER_P, SORTER_I, SORTER_D, SORTER_F
                )
        );

        beamBreak = hw.get(DigitalChannel.class, "outtakeBeamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);
        lastBeamClear = beamBreak.getState();

        colorSensor = hw.get(ColorSensor.class, "sorterColorSensor");

        statusLEDServo = hw.get(Servo.class, "statusLED");
        statusLED = new StatusLED_RGB(statusLEDServo);

        // default idle state
        statusLED.setState(StatusLED_RGB.LEDState.WHITE);

        for (int i = 0; i < SLOT_COUNT; i++)
            slots[i] = new Slot();
    }

    // ================= PUBLIC API =================

    /** Rotate one slot forward for intake */
    public void rotateForIntake() {
        if (busy) return;
        commandRotationDeg(120.0);
    }

    /** Rotate closest slot of given color to outtake (slot 0) */
    public void rotateClosestToOuttake(BallColor color) {
        if (busy) return;

        int bestSlot = findClosestSlotToPlane(color, 180);
        if (bestSlot == -1) return;

        double slotAngle =
                normalize(wheelAngleDeg + bestSlot * 120.0);

        double delta = 180.0 - slotAngle;

        // shortest path
        if (delta > 180) delta -= 360;
        if (delta < -180) delta += 360;

        commandRotationDeg(delta);
    }

    /** Mark color detected at intake */
    public void markPendingBall(BallColor color) {
        pendingBall = color;
    }

    public boolean isBusy() {
        return busy;
    }

    public BallColor getSlotColor(int slot) {
        return slots[slot].color;
    }

    // ================= UPDATE LOOP =================
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        updateBeamBreak();

        packet.put("sorter/currentSlot", currentSlot);
        packet.put("sorter/targetSlot", targetSlot);
        packet.put("sorter/busy", busy);

        packet.put("color/r", colorSensor.red());
        packet.put("color/g", colorSensor.green());
        packet.put("color/b", colorSensor.blue());
        packet.put("color/sum", colorSensor.red() + colorSensor.green() + colorSensor.blue());


        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        updateStatusLED();

        if (!busy) return;

        if (!motor.isBusy()) {
            motor.setPower(0);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            finalizeRotation(pendingDeltaDeg);
            pendingDeltaDeg = 0;
            busy = false;
        }
    }

    private void updateStatusLED() {
        if (statusLED == null) return;

        if (busy) {
            statusLED.setState(StatusLED_RGB.LEDState.YELLOW);
            return;
        }

        int intakeSlot = getSlotAtPlane(0);
        if (intakeSlot != -1) {
            BallColor sensed = readColorSensor();
            if (sensed != BallColor.NONE) {
                statusLED.setState(
                        sensed == BallColor.GREEN
                                ? StatusLED_RGB.LEDState.GREEN
                                : StatusLED_RGB.LEDState.PURPLE
                );
                return;
            }
        }

        int outtakeSlot = getSlotAtPlane(180);
        if (outtakeSlot != -1) {
            BallColor stored = slots[outtakeSlot].color;
            if (stored != BallColor.NONE) {
                statusLED.setState(
                        stored == BallColor.GREEN
                                ? StatusLED_RGB.LEDState.GREEN
                                : StatusLED_RGB.LEDState.PURPLE
                );
                return;
            }
        }

        statusLED.setState(StatusLED_RGB.LEDState.WHITE);
    }

    private BallColor readColorSensor() {
        if (colorSensor == null) return BallColor.NONE;

        double r = colorSensor.red();
        double g = colorSensor.green();
        double b = colorSensor.blue();

        double sum = r + g + b;

        if (sum < COLOR_MIN_SUM) {
            return BallColor.NONE;
        }

        // Avoid divide-by-zero
        if (r < 1) r = 1;
        if (g < 1) g = 1;
        if (b < 1) b = 1;

        double gOverR = g / r;
        double gOverB = g / b;

        double rOverG = r / g;
        double rOverB = r / b;

        if (gOverR > GREEN_G_OVER_R && gOverB > GREEN_G_OVER_B) {
            return BallColor.GREEN;
        }

        if (rOverG > PURPLE_R_OVER_G && rOverB > PURPLE_R_OVER_B) {
            return BallColor.PURPLE;
        }

        return BallColor.NONE;
    }

    // ================= INTERNAL =================

    private double pendingDeltaDeg = 0;

    private void commandRotationDeg(double deltaDeg) {
        pendingDeltaDeg = deltaDeg;

        int ticks = (int)(deltaDeg / 360.0 * TICKS_PER_REV);

        motor.setTargetPosition(motor.getCurrentPosition() + ticks);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setVelocity(SORTER_VELOCITY);

        busy = true;
    }

    private void finalizeRotation(double deltaDeg) {
        wheelAngleDeg = normalize(wheelAngleDeg + deltaDeg);

        // store pending ball into the slot now at intake plane (0°)
        int intakeSlot = getSlotAtPlane(0);
        if (intakeSlot != -1 && pendingBall != BallColor.NONE && slots[intakeSlot].color == BallColor.NONE) {
            slots[intakeSlot].color = pendingBall;
            pendingBall = BallColor.NONE;
        }
    }

    private void updateBeamBreak() {
        boolean beamClear = beamBreak.getState();

        // falling edge = ball ejected
        if (lastBeamClear && !beamClear) {
            onBallEjected();
        }

        lastBeamClear = beamClear;
    }

    private void onBallEjected() {
        int outtakeSlot = getSlotAtPlane(180);
        if (outtakeSlot != -1) {
            slots[outtakeSlot].color = BallColor.NONE;
        }
    }

    private int findClosestSlot(BallColor color) {
        int best = -1;
        int bestDist = Integer.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (slots[i].color == color) {
                int d = Math.abs(deltaSlots(currentSlot, i));
                if (d < bestDist) {
                    bestDist = d;
                    best = i;
                }
            }
        }
        return best;
    }

    private int deltaSlots(int from, int to) {
        int d = to - from;
        if (d > SLOT_COUNT / 2) d -= SLOT_COUNT;
        if (d < -SLOT_COUNT / 2) d += SLOT_COUNT;
        return d;
    }

    private int mod(int x, int m) {
        return (x % m + m) % m;
    }

    private double normalize(double deg) {
        deg %= 360;
        if (deg < 0) deg += 360;
        return deg;
    }

    private int getSlotAtPlane(double planeDeg) {
        for (int i = 0; i < SLOT_COUNT; i++) {
            double angle = normalize(wheelAngleDeg + i * 120.0);
            if (Math.abs(angle - planeDeg) < 10) { // tolerance
                return i;
            }
        }
        return -1;
    }

    private int findClosestSlotToPlane(BallColor color, double planeDeg) {
        int best = -1;
        double bestDist = Double.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            if (slots[i].color == color) {
                double angle = normalize(wheelAngleDeg + i * 120.0);
                double d = Math.abs(angle - planeDeg);
                d = Math.min(d, 360 - d);
                if (d < bestDist) {
                    bestDist = d;
                    best = i;
                }
            }
        }
        return best;
    }

    /** Rotate the currently selected pocket to the intake plane (0°) */
    public void rotateIntakePocketToOuttakePlane() {
        if (busy) return;
        commandRotationDeg(180.0); // or -180, but pick one direction preference
    }


    /** Rotate the currently selected pocket to the outtake plane (180°) */
    public void rotateOuttakePocketToIntakePlane() {
        if (busy) return;
        commandRotationDeg(-180.0);
    }

    /** Manually rotate sorter by a fixed number of slots (positive or negative) */
    public void rotateBySlots(int slotDelta) {
        if (busy) return;
        commandRotationDeg(slotDelta * 120.0);
    }

}
