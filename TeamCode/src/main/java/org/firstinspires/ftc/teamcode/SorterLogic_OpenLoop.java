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
public class SorterLogic_OpenLoop {

    // ================= HARD CONSTANTS =================
    public static int TICKS_PER_REV = 384;

    public static int STEP_60 = TICKS_PER_REV / 6;

    public static double SORTER_POWER = 0.4;

    // ================= PIDF ======================
    public static double SORTER_P = 2.5;
    public static double SORTER_I = 0.1;
    public static double SORTER_D = 0.2;
    public static double SORTER_F = 0.5;

    // ================= HARDWARE =================
    private DcMotorEx motor;
    private Telemetry telemetry;
    private final ElapsedTime loopTimer = new ElapsedTime();

    // ================= STATE =================
    private int targetPosition = 0;
    private boolean isBusy = false;

    // ================= INIT =================
    public void init(HardwareMap hw, Telemetry telemetry) {
        this.telemetry = telemetry;
        motor = hw.get(DcMotorEx.class, "sorterMotor");

        motor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        PIDFCoefficients pidfNew = new PIDFCoefficients(SORTER_P, SORTER_I, SORTER_D, SORTER_F);
        motor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER, pidfNew);

        loopTimer.reset();
    }

    // ================= COMMAND =================
    public void moveForward60() {
        commandMove(+STEP_60);
    }

    public void moveBack60() {
        commandMove(-STEP_60);
    }

    private void commandMove(int deltaTicks) {
        if (isBusy) return; // prevent overlapping commands

        targetPosition = motor.getCurrentPosition() + deltaTicks;

        motor.setTargetPosition(targetPosition);
        motor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        motor.setPower(SORTER_POWER);

        isBusy = true;
    }

    // ================= UPDATE LOOP =================
// ================= UPDATE LOOP =================
    public void update() {
        TelemetryPacket packet = new TelemetryPacket();

        int currentPos = motor.getCurrentPosition();
        int error = targetPosition - currentPos;

        // ----- Dashboard graph data -----
        packet.put("sorter/error_ticks", error);
        packet.put("sorter/current_pos", currentPos);
        packet.put("sorter/target_pos", targetPosition);
        packet.put("sorter/isBusy", isBusy);

        FtcDashboard.getInstance().sendTelemetryPacket(packet);

        if (!isBusy) return;

        // Motor finished moving
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

}
