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
    public static final double TICKS_PER_REV = 537.4;
    private static final int SLOT_COUNT = 3;

    public static double POS_P = 0.01;     // power per tick of error
    public static double POS_I = 0.0;
    public static double POS_D = 0.006;
    public static double POS_F = 0.08;      // 0..1 constant push toward target

    private double posIntegral = 0;
    private double lastPosError = 0;

    // encoder ticks for each pocket at INTAKE plane
    public static int[] INTAKE_TICKS = {
            0, // pocket 0
            176, // pocket 1
            373 // pocket 2
    };

    // encoder ticks for each pocket at OUTTAKE plane
    public static int[] OUTTAKE_TICKS = {
            -259, // pocket 0
            -80, // pocket 1
            114 // pocket 2
    };

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

    // ================= STATE =================
    private boolean busy = false;
    private boolean lastBeamClear = true;
    private BallColor pendingBall = BallColor.NONE;
    private int targetTicks = 0;
    public static int POS_DONE_TICKS = 2;   // settle window
    public static double POS_MAX_POWER = 0.5;
    public static double POS_I_MAX = 2000; // tune


    // ================= INIT =================
    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = hw.get(DcMotorEx.class, "sorterMotor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

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

        targetTicks = motor.getCurrentPosition(); // after reset, this is 0
        posIntegral = 0;
        lastPosError = 0;
        busy = false;
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

        int currentTicks = motor.getCurrentPosition();
        int errorTicks = targetTicks - currentTicks;
        double errorDeg = errorTicks / TICKS_PER_REV * 360.0;

        packet.put("sorter/targetTicks", targetTicks);
        packet.put("sorter/currentTicks", currentTicks);
        packet.put("sorter/errorTicks", errorTicks);
        packet.put("sorter/errorDeg", errorDeg);

        packet.put("color/r", colorSensor.red());
        packet.put("color/g", colorSensor.green());
        packet.put("color/b", colorSensor.blue());
        packet.put("color/sum", colorSensor.red() + colorSensor.green() + colorSensor.blue());



        updateStatusLED();

        int current = motor.getCurrentPosition();
        int error = targetTicks - current;

        // PID terms
        if (Math.abs(error) <= POS_DONE_TICKS) {
            posIntegral = 0; // or slowly decay
        } else {
            posIntegral += error;
            posIntegral = Math.max(-POS_I_MAX, Math.min(POS_I_MAX, posIntegral));
        }

        double derivative = error - lastPosError;

        double pid =
                POS_P * error +
                        POS_I * posIntegral +
                        POS_D * derivative;

        // F term for position: constant assist toward target
        double ff = 0.0;
        if (Math.abs(error) > POS_DONE_TICKS) {
            ff = Math.copySign(POS_F, error); // pushes toward target
        }

        double output = pid + ff;

        // Clamp
        output = Math.max(-POS_MAX_POWER, Math.min(POS_MAX_POWER, output));

        motor.setPower(output);
        lastPosError = error;

        // Telemetry for graphing
        packet.put("sorter/pid", pid);
        packet.put("sorter/ff", ff);
        packet.put("sorter/output", output);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (busy && Math.abs(error) <= POS_DONE_TICKS) {
            finalizeMove();     // encoder-based
            busy = false;
            posIntegral = 0;    // prevent windup while holding
        }
    }

    private void updateStatusLED() {
        if (statusLED == null) return;

        if (busy) {
            statusLED.setState(StatusLED_RGB.LEDState.YELLOW);
            return;
        }

        int intakeSlot = getPocketClosestTo(INTAKE_TICKS);
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

        int outtakeSlot = getPocketClosestTo(OUTTAKE_TICKS);
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
    public void movePocketToIntake(int pocket) {
        if (busy || !validPocket(pocket)) return;
        targetTicks = INTAKE_TICKS[pocket];
        startMove();
    }

    public void movePocketToOuttake(int pocket) {
        if (busy || !validPocket(pocket)) return;
        targetTicks = OUTTAKE_TICKS[pocket];
        startMove();
    }

    public int getPocketClosestTo(int[] tickTable) {
        int cur = motor.getCurrentPosition();
        int best = -1;
        int bestErr = Integer.MAX_VALUE;

        for (int i = 0; i < SLOT_COUNT; i++) {
            int err = Math.abs(tickTable[i] - cur);
            if (err < bestErr) {
                bestErr = err;
                best = i;
            }
        }
        return best;
    }

    private void startMove() {
        posIntegral = 0;
        lastPosError = 0;
        busy = true;
    }

    private void finalizeMove() {
        // if a ball was just detected at intake, commit it to whichever pocket is at intake now
        int intakeSlot = getPocketClosestTo(INTAKE_TICKS);
        if (intakeSlot != -1 && pendingBall != BallColor.NONE && slots[intakeSlot].color == BallColor.NONE) {
            slots[intakeSlot].color = pendingBall;
            pendingBall = BallColor.NONE;
        }
    }

    private boolean validPocket(int p) { return p >= 0 && p < SLOT_COUNT; }

    public int getPocketWithColor(BallColor color) {
        for (int i = 0; i < SLOT_COUNT; i++) {
            if (slots[i].color == color) {
                return i;
            }
        }
        return -1;
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
        int outtakeSlot = getPocketClosestTo(OUTTAKE_TICKS);
        if (outtakeSlot != -1) {
            slots[outtakeSlot].color = BallColor.NONE;
        }
    }

    public BallColor getPendingBall() {
        return pendingBall;
    }

    public BallColor[] getSlotColors() {
        BallColor[] c = new BallColor[SLOT_COUNT];
        for (int i = 0; i < SLOT_COUNT; i++) {
            c[i] = slots[i].color;
        }
        return c;
    }


}
