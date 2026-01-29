package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

public class MotifShoot {

    private final Mechanisms mechanisms;

    private String motifPattern;
    private int motifIndex = 0;
    private boolean active = false;

    private final ElapsedTime timer = new ElapsedTime();

    private enum State {
        MOVE_TO_OUTTAKE,
        WAIT_FOR_SORTER,
        SHOOT,
        WAIT_AFTER_SHOT,
        DONE
    }

    private State state = State.DONE;

    public MotifShoot(Mechanisms mechanisms) {
        this.mechanisms = mechanisms;
    }

    // ================= ENTRY POINT =================
    public void start(String motifPattern) {
        this.motifPattern = motifPattern; // e.g. "GPP"
        motifIndex = 0;
        active = true;
        state = State.MOVE_TO_OUTTAKE;

        // visual feedback
        mechanisms.sorter.triggerMotifLockedFlash();
    }

    public boolean isFinished() {
        return state == State.DONE;
    }

    // ================= CALL IN AUTO LOOP =================
    public void update() {
        if (!active) return;

        switch (state) {

            case MOVE_TO_OUTTAKE: {
                char c = motifPattern.charAt(motifIndex);

                FinalSorter.BallColor wanted =
                        (c == 'G')
                                ? FinalSorter.BallColor.GREEN
                                : FinalSorter.BallColor.PURPLE;

                int pocket = mechanisms.sorter.getPocketWithColor(wanted);

                // safety fallback (should never happen if autos correct)
                if (pocket == -1) pocket = 0;

                mechanisms.sorter.movePocketToOuttake(pocket);
                state = State.WAIT_FOR_SORTER;
                break;
            }

            case WAIT_FOR_SORTER:
                if (!mechanisms.isSorterBusy()) {
                    state = State.SHOOT;
                }
                break;

            case SHOOT:
                mechanisms.ejectBall();
                timer.reset();
                state = State.WAIT_AFTER_SHOT;
                break;

            case WAIT_AFTER_SHOT:
                // allow beam-break + kicker cycle
                if (timer.seconds() > 0.6) {
                    motifIndex++;

                    if (motifIndex >= 3) {
                        state = State.DONE;
                        active = false;
                    } else {
                        state = State.MOVE_TO_OUTTAKE;
                    }
                }
                break;

            case DONE:
                active = false;
                break;
        }
    }
}
