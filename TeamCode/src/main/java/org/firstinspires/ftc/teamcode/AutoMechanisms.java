package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class AutoMechanisms {

    private final Mechanisms mech;
    private final ElapsedTime timer = new ElapsedTime();

    // ---------- STATE ----------
    private boolean shotInProgress = false;

    public AutoMechanisms(Mechanisms mechanisms) {
        this.mech = mechanisms;
    }

    // ===========================
    // INTAKE
    // ===========================

    /** Intake balls inward */
    public void intakeIn(double power) {
        mech.engageIntake(power, true);
    }

    /** Reverse intake to eject */
    public void intakeOut(double power) {
        mech.engageIntake(power, false);
    }

    /** Stop intake */
    public void stopIntake() {
        mech.disengageIntake();
    }

    // ===========================
    // OUTTAKE / SHOOTER
    // ===========================

    /** Spin shooter flywheel */
    public void spinUpShooter(double speed) {
        mech.engageOuttake(speed);
    }

    /** Stop shooter flywheel */
    public void stopShooter() {
        mech.disengageOuttake();
    }

    /** Set ramp angle */
    public void setRampAngle(double pos) {
        mech.setRampAngle(pos);
    }

    // ===========================
    // FIRING
    // ===========================

    /** Fire exactly one ball */
    public void fireOnce() {
        if (!shotInProgress) {
            mech.ejectBall();
            shotInProgress = true;
            timer.reset();
        }
    }

    /** Returns true when kick cycle is done */
    public boolean isShotComplete() {
        if (!shotInProgress) return true;

        if (timer.seconds() >= Mechanisms.KICK_DURATION + 0.05) {
            shotInProgress = false;
            return true;
        }
        return false;
    }

    // ===========================
    // CONVENIENCE
    // ===========================

    /** Full auto shot helper */
    public void shoot(double speed, double rampAngle) {
        spinUpShooter(speed);
        setRampAngle(rampAngle);
        fireOnce();
    }
}
