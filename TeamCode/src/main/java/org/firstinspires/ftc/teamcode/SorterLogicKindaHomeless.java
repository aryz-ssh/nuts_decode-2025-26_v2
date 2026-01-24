/*package org.firstinspires.ftc.teamcode;

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
public class SorterLogicKindaHomeless {

    // ================= HARD CONSTANTS =================
    public static double TICKS_PER_REV = 384.5;

    // public static int STEP_60 = (int) Math.round(TICKS_PER_REV / 6.0);
    public static double SORTER_POWER = 0.4;

    // ================= PIDF ======================
    // TODO: TUNE PIDF VALUES!!
    public static double SORTER_P = 2.5;
    public static double SORTER_I = 0.0;
    public static double SORTER_D = 0.2;
    public static double SORTER_F = 11;

    // ================= HARDWARE =================
    private DcMotorEx motor;
    private DigitalChannel beamBreak;   // true = no ball, false = ball present
    ColorSensor sorterColorSensor;
    private Servo statusLEDServo;
    private StatusLED_RGB statusLED;
    private Telemetry telemetry;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ================= STATE =================
    private int targetPosition = 0;
    private boolean isBusy = false;
    private int slot = 0;   // 0..5, assumes correct alignment at start
    public enum BallColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }
    private BallColor currentBallColor = BallColor.UNKNOWN;

    private boolean lastBeamState = true;   // beam initially unbroken
    private boolean ballJustEjected = false;

    private int greenBallCount = 0;
    private int purpleBallCount = 0;


    // ================= INIT =================
    public void init(HardwareMap hw, Telemetry telemetry) {
        // ========= SORTER MOTOR =========
        this.telemetry = telemetry;
        motor = hw.get(DcMotorEx.class, "sorterMotor");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(SORTER_P, SORTER_I, SORTER_D, SORTER_F);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        // ========= BEAM BREAK =========
        beamBreak = hw.get(DigitalChannel.class, "outtakeBeamBreak");
        beamBreak.setMode(DigitalChannel.Mode.INPUT);

        lastBeamState = beamBreak.getState();

        // ========= COLOR SENSOR =========
        sorterColorSensor = hw.get(ColorSensor.class, "sorterColorSensor");

        // ========= STATUS RGB =========
        statusLEDServo = hw.get(Servo.class, "statusLED");
        statusLED = new StatusLED_RGB(statusLEDServo);
        statusLED.setState(StatusLED_RGB.LEDState.WHITE);

        loopTimer.reset();
    }

    // ================= COMMAND =================
    public void moveForward60() {
        gotoSlot(slot + 1);
    }

    public void moveBack60() {
        gotoSlot(slot - 1);
    }

    private void gotoSlot(int newSlot) {
        if (isBusy) return;

        slot = (newSlot % 6 + 6) % 6;

        targetPosition = (int) Math.round(slot * (TICKS_PER_REV / 6.0));

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(SORTER_POWER);

        isBusy = true;
    }

    private void updateBeamBreak() {
        boolean beamClear = beamBreak.getState(); // true = clear, false = blocked
        ballJustEjected = false;

        // Detect falling edge: clear -> blocked
        if (lastBeamState && !beamClear) {
            ballJustEjected = true;
            onBallEjected();
        }

        lastBeamState = beamClear;
    }

    private void onBallEjected() {
        switch (currentBallColor) {
            case GREEN:  greenBallCount++;  break;
            case PURPLE: purpleBallCount++; break;
            default: break;
        }

        currentBallColor = BallColor.UNKNOWN;
    }

    // ================= UPDATE LOOP =================
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        updateBeamBreak();

        int currentPos = motor.getCurrentPosition();
        int error = targetPosition - currentPos;

        // ----- Dashboard graph data -----
        packet.put("sorter/error_ticks", error);
        packet.put("sorter/current_pos", currentPos);
        packet.put("sorter/target_pos", targetPosition);
        packet.put("sorter/isBusy", isBusy);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (!isBusy) return;

        if (!motor.isBusy()) {
            motor.setPower(0);
            motor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            isBusy = false;
        }
    }

    // ================= UTILITY =================
    public boolean isBusy() {
        return isBusy;
    }

    // ---------- TELEMETRY ACCESSORS ----------
    public int getCurrentPosition() {
        return motor.getCurrentPosition();
    }

    public int getTargetPosition() {
        return targetPosition;
    }

    public int getErrorTicks() {
        return targetPosition - motor.getCurrentPosition();
    }

}*/
