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

    public static double POS_P = 0.0028;     // power per tick of error
    public static double POS_I = 0.0;
    public static double POS_D = 0.001;
    public static double POS_F = 0.07;      // 0..1 constant push toward target

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

// ================= COLOR CLASSIFICATION (DATA-DRIVEN) =================

    // Chroma-based presence detection
    public static int CHROMA_ON  = 90;   // detects holes reliably
    public static int CHROMA_OFF = 60;   // hysteresis floor

    // GREEN detection (very strong separation)
    public static double GREEN_G_OVER_R = 2.0;  // hole min was 2.29
    public static double GREEN_G_OVER_B = 1.2;  // hole min was 1.30

    // PURPLE detection (blue-dominant)
    public static double PURPLE_B_OVER_G = 1.1; // hole min was ~1.17
    public static double PURPLE_B_OVER_R = 1.5; // conservative, solid ~1.9

    private boolean ballPresent = false;
    private boolean lastBallPresent = false;

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
    private boolean autoMode = false;
    private BallColor pendingBall = BallColor.NONE;
    private int targetTicks = 0;
    public static int POS_DONE_TICKS = 6;   // settle window
    public static double POS_MAX_POWER = 0.5;
    public static double POS_I_MAX = 2000; // tune

    // ================= BUSY TIMEOUT =================
    public static long BUSY_TIMEOUT_MS = 800; // dashboard tunable
    private long busyStartTimeMs = 0;
    private int lastBusyTicks = 0;

    // ================= EJECTION FLASH =================
    public static long EJECT_FLASH_MS = 200; // tunable
    private long lastEjectTimeMs = -1;

    // ================= MOTIF LOCK FLASH =================
    private boolean motifFlashActive = false;
    private long motifFlashStartMs = 0;
    public static long MOTIF_FLASH_MS = 200;

    // Dashboard control
    public static boolean DASH_ENABLED = true;
    public static long DASH_PERIOD_MS = 75; // optional throttle

    private long lastDashMs = 0;

    // ================= INIT =================
    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;

        motor = hw.get(DcMotorEx.class, "sorterMotor");
        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        // motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
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
        boolean currentPresent = detectPresence(
                colorSensor.red(),
                colorSensor.green(),
                colorSensor.blue()
        );

        // ================= AUTO MODE: CYCLE ON CONFIRMED COLOR =================
        if (autoMode && !busy) {

            BallColor detected = readColorSensor();  // SAME logic as LED

            if (detected != BallColor.NONE) {

                int intakeSlot = getPocketClosestTo(INTAKE_TICKS);

                // Only act if the intake pocket is empty
                if (intakeSlot != -1 && slots[intakeSlot].color == BallColor.NONE) {

                    // Commit ball to this pocket
                    slots[intakeSlot].color = detected;

                    // Immediately advance to the next available pocket
                    advanceIntakePocketIfPossible();
                }
            }
        }

        updateBeamBreak();


        int currentTicks = motor.getCurrentPosition();
        int errorTicks = targetTicks - currentTicks;
        double errorDeg = errorTicks / TICKS_PER_REV * 360.0;

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

        long now = System.currentTimeMillis();

        if (DASH_ENABLED && now - lastDashMs >= DASH_PERIOD_MS) {
            TelemetryPacket packet = new TelemetryPacket();

            packet.put("sorter/targetTicks", targetTicks);
            packet.put("sorter/currentTicks", currentTicks);
            packet.put("sorter/errorTicks", errorTicks);
            packet.put("sorter/errorDeg", errorDeg);

            packet.put("color/r", colorSensor.red());
            packet.put("color/g", colorSensor.green());
            packet.put("color/b", colorSensor.blue());
            packet.put("color/sum", colorSensor.red() + colorSensor.green() + colorSensor.blue());

            packet.put("sorter/targetTicks", targetTicks);
            packet.put("sorter/currentTicks", motor.getCurrentPosition());
            packet.put("sorter/errorTicks", error);
            packet.put("sorter/output", output);

            FtcDashboard.getInstance().sendTelemetryPacket(packet);
            lastDashMs = now;
        }

        if (busy) {
            now = System.currentTimeMillis();
            int curTicks = motor.getCurrentPosition();

            // Detect lack of movement
            boolean stalled = Math.abs(curTicks - lastBusyTicks) < 2;

            if (stalled && now - busyStartTimeMs > BUSY_TIMEOUT_MS) {
                busy = false;
                posIntegral = 0;
            }

            lastBusyTicks = curTicks;
        }

        if (busy && Math.abs(error) <= POS_DONE_TICKS) {
            busy = false;
            posIntegral = 0;    // prevent windup while holding
        }
    }

    private void updateStatusLED() {
    // --- MOTIF LOCK FLASH (CYAN) ---
        if (motifFlashActive) {
            long now = System.currentTimeMillis();

            if (now - motifFlashStartMs < MOTIF_FLASH_MS) {
                statusLED.setState(StatusLED_RGB.LEDState.CYAN);
                return;
            } else {
                motifFlashActive = false;
                // fall through to normal LED behavior
            }
        }

        if (statusLED == null) return;

        long now = System.currentTimeMillis();
        // --- EJECTION ALERT OVERRIDE ---
        if (lastEjectTimeMs > 0) {
            if (now - lastEjectTimeMs < EJECT_FLASH_MS) {
                statusLED.setState(StatusLED_RGB.LEDState.RED);
                return;
            } else {
                // flash window expired → clear latch
                lastEjectTimeMs = -1;
            }
        }

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
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        if (!detectPresence(r, g, b)) {
            return BallColor.NONE;
        }

        double rr = Math.max(1, r);
        double gg = Math.max(1, g);
        double bb = Math.max(1, b);

        // GREEN: strongly green-dominant
        if (gg / rr > GREEN_G_OVER_R && gg / bb > GREEN_G_OVER_B) {
            return BallColor.GREEN;
        }

        // PURPLE: blue-dominant (NOT red-dominant)
        if (bb / gg > PURPLE_B_OVER_G && bb / rr > PURPLE_B_OVER_R) {
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

        busyStartTimeMs = System.currentTimeMillis();
        lastBusyTicks = motor.getCurrentPosition();
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

    private void commitPendingBallToIntake() {
        if (pendingBall == BallColor.NONE) return;

        int intakeSlot = getPocketClosestTo(INTAKE_TICKS);

        if (intakeSlot != -1 && slots[intakeSlot].color == BallColor.NONE) {
            slots[intakeSlot].color = pendingBall;
            pendingBall = BallColor.NONE;

            // 🔥 AUTO ADVANCE TRIGGER HERE 🔥
            if (autoMode) {
                advanceIntakePocketIfPossible();
            }
        }
    }

    private boolean detectPresence(int r, int g, int b) {
        int max = Math.max(r, Math.max(g, b));
        int min = Math.min(r, Math.min(g, b));
        int chroma = max - min;

        if (!ballPresent && chroma > CHROMA_ON) {
            ballPresent = true;
        } else if (ballPresent && chroma < CHROMA_OFF) {
            ballPresent = false;
        }

        return ballPresent;
    }

    private void updateBeamBreak() {
        // Normalize: true = clear, false = blocked
        boolean beamClearNow = !beamBreak.getState();

        // FALLING EDGE: clear → blocked (ball ENTERS beam)
        if (lastBeamClear && !beamClearNow) {
            onBallEjected();
        }

        lastBeamClear = beamClearNow;
    }

    public void onBallEjected() {
        int outtakeSlot = getPocketClosestTo(OUTTAKE_TICKS);
        if (outtakeSlot != -1) {
            slots[outtakeSlot].color = BallColor.NONE;
        }

        // trigger red flash
        lastEjectTimeMs = System.currentTimeMillis();
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

    // ================= AUTOS PEOPLE, AUTOS =================
    public void setAutoMode(boolean enabled) {
        autoMode = enabled;
    }

    public int getPocketWithAnyBall() {
        for (int i = 0; i < SLOT_COUNT; i++) {
            if (slots[i].color != BallColor.NONE) {
                return i;
            }
        }
        return -1;
    }

    private void advanceIntakePocketIfPossible() {
        if (busy) return;

        int currentIntake = getPocketClosestTo(INTAKE_TICKS);
        if (currentIntake == -1) return;

        // Check if current intake pocket is now occupied
        if (slots[currentIntake].color == BallColor.NONE) return;

        // Find next empty pocket
        for (int i = 1; i <= SLOT_COUNT; i++) {
            int next = (currentIntake + i) % SLOT_COUNT;

            if (slots[next].color == BallColor.NONE) {
                movePocketToIntake(next);
                return;
            }
        }

        // All slots full → do nothing
    }

    public void triggerMotifLockedFlash() {
        motifFlashActive = true;
        motifFlashStartMs = System.currentTimeMillis();
    }

    public void forceSetSlotColors(BallColor[] colors) {
        if (colors == null || colors.length != SLOT_COUNT) return;

        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i].color = colors[i];
        }
    }

    public void clearAllSlots() {
        for (int i = 0; i < SLOT_COUNT; i++) {
            slots[i].color = BallColor.NONE;
        }
    }

    // ================= DEBUG / TUNING ACCESS =================

    public int getColorR() {
        return colorSensor.red();
    }

    public int getColorG() {
        return colorSensor.green();
    }

    public int getColorB() {
        return colorSensor.blue();
    }

    /** Raw detected color at intake (no slot commit) */
    public BallColor getDetectedColorAtIntake() {
        return readColorSensor();
    }

    /** Presence state (for tuning hysteresis) */
    public boolean isBallPresent() {
        return ballPresent;
    }
}
