package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotifShoot {

    private final Mechanisms mechanisms;
    private final AprilTagLimelight limelight;

    private String motif;
    private int motifIndex = 0;
    private boolean active = false;

    private final ElapsedTime timer = new ElapsedTime();

    private enum State {
        MOVE_TO_POCKET,
        WAIT_FOR_SORTER,
        SHOOT,
        WAIT_FOR_CONFIRM,
        DONE
    }

    private State state = State.DONE;

    public MotifShoot(Mechanisms mechanisms, AprilTagLimelight limelight) {
        this.mechanisms = mechanisms;
        this.limelight = limelight;
    }

    // ---------------- ENTRY POINT (AUTO) ----------------
    public void startMotifShoot() {
        motif = limelight.getMotif();   // "GPP", "PGP", "PPG"
        motifIndex = 0;
        active = true;
        state = State.MOVE_TO_POCKET;
    }

    public boolean isFinished() {
        return state == State.DONE;
    }

    // ---------------- CALL IN AUTO LOOP ----------------
    public void update() {
        if (!active) return;

        switch (state) {

            case MOVE_TO_POCKET: {
                char desired = motif.charAt(motifIndex);

                SorterLogicColor.BallColor wanted =
                        (desired == 'G')
                                ? SorterLogicColor.BallColor.GREEN
                                : SorterLogicColor.BallColor.PURPLE;

                Integer pocket = mechanisms.sorterLogic.getPocketWithColor(wanted);

                // Safety fallback (should not happen if preload/intake worked)
                if (pocket == null) pocket = 1;

                mechanisms.sorterGoToOuttake(pocket);
                mechanisms.setShotPocket(pocket);

                state = State.WAIT_FOR_SORTER;
                break;
            }

            case WAIT_FOR_SORTER:
                if (!mechanisms.isSorterMoving()) {
                    state = State.SHOOT;
                }
                break;

            case SHOOT:
                mechanisms.ejectBall();   // starts beam-break verification
                timer.reset();
                state = State.WAIT_FOR_CONFIRM;
                break;

            case WAIT_FOR_CONFIRM:
                // Either beam-break confirms OR timeout fails safely
                if (timer.seconds() > 0.9) {
                    motifIndex++;
                    if (motifIndex >= 3) {
                        state = State.DONE;
                        active = false;
                    } else {
                        state = State.MOVE_TO_POCKET;
                    }
                }
                break;

            case DONE:
                active = false;
                break;
        }
    }
}
