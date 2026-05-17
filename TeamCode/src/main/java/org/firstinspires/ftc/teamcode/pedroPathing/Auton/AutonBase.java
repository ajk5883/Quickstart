package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.ShootSequencer;

import java.util.Collections;
import java.util.List;

/**
 * AutonBase – abstract autonomous OpMode that executes an ordered sequence of
 * {@link PathStep}s supplied by a concrete subclass via {@link #getConfig()}.
 *
 * <p>Subclasses only need to:
 * <ol>
 *   <li>Add an {@code @Autonomous} annotation.</li>
 *   <li>Implement {@link #getConfig()} to return an {@link AutonConfig} whose
 *       {@link AutonConfig.StepsFactory} builds all poses and
 *       {@link com.pedropathing.paths.PathChain}s for that specific routine.</li>
 * </ol>
 *
 * State machine
 * ─────────────
 *   INIT_STEP       – Reads the current step's {@link PoseType} and applies the
 *                     corresponding subsystem action (see below).
 *   FOLLOWING_PATH  – Waits for the follower to finish travelling the step's path.
 *                     Transitions to INIT_STEP when the path is complete.
 *   SHOOTING        – Waits for the shoot-window timer.  Transitions to the next
 *                     step when the timer elapses.
 *   DONE            – Terminal state; all steps completed (robot is parked).
 *
 * Subsystem actions by PoseType
 * ─────────────────────────────
 *   STARTING     – Enables keepShooterRunningAfterShoot; advances immediately.
 *   SCORING      – Stops intake, sets hood, starts timed shoot sequence, enters SHOOTING.
 *   PICKUP_START – Starts intake; advances immediately (sweep begins).
 *   PICKUP_END   – Stops intake; advances immediately.
 *   PARKING      – Stops all subsystems; enters DONE.
 */
public abstract class AutonBase extends OpMode {

    // ── State machine ─────────────────────────────────────────────────────────

    private enum RunState {
        INIT_STEP,
        FOLLOWING_PATH,
        SHOOTING,
        DONE
    }

    // ── Hardware ──────────────────────────────────────────────────────────────

    private Follower       follower;
    private ShootSequencer shootSequencer;

    // ── Runtime state ─────────────────────────────────────────────────────────

    private AutonConfig    config;
    private List<PathStep> steps = Collections.emptyList();
    private int            stepIndex;
    private RunState       runState         = RunState.DONE;
    private long           shootSequenceEndMs;

    // ── Abstract contract ─────────────────────────────────────────────────────

    /**
     * Return the full {@link AutonConfig} for this specific autonomous variant.
     * The config's {@link AutonConfig.StepsFactory} receives the constructed
     * {@link Follower} and must return the complete, ordered list of
     * {@link PathStep}s to execute.
     */
    protected abstract AutonConfig getConfig();

    // ── State machine helpers ─────────────────────────────────────────────────

    /**
     * Applies the subsystem action for {@code step.poseType}.
     * Either enters SHOOTING (for SCORING) or calls {@link #nextStep()} immediately.
     */
    private void applyStepAction(PathStep step) {
        switch (step.poseType) {

            case STARTING:
                // Shooter stays spinning between shots for the whole autonomous run.
                shootSequencer.setShooterRunningAfterShoot(true);
                nextStep();
                break;

            case SCORING:
                // Gate opens; timed shoot sequence runs; state machine waits.
                // Per-step values override the global config defaults when set.
                double rpm  = Double.isNaN(step.shootTargetVelocity)
                        ? config.shootTargetVelocity : step.shootTargetVelocity;
                double hood = Double.isNaN(step.shootHoodPosition)
                        ? config.shootHoodPosition   : step.shootHoodPosition;
                shootSequencer.stopIntake();
                shootSequencer.setHoodPosition(hood);
                shootSequencer.startShootingSequence(
                        config.shootDurationMs,
                        rpm,
                        config.shootVelocityThreshold);
                shootSequenceEndMs = System.currentTimeMillis() + config.shootSequenceWaitMs;
                runState = RunState.SHOOTING;
                break;

            case PICKUP_START:
                // Intake on; robot immediately begins the sweep path to PICKUP_END.
                shootSequencer.startIntake();
                nextStep();
                break;

            case PICKUP_END:
                // Intake off; robot immediately heads toward the next scoring pose.
                shootSequencer.stopIntake();
                nextStep();
                break;

            case PARKING:
                // All done — shut everything down.
                shootSequencer.setShooterRunningAfterShoot(false);
                shootSequencer.stopIntake();
                shootSequencer.stopShootingSequence();
                runState = RunState.DONE;
                break;
        }
    }

    /**
     * Advances to the next step, starts following its path, and updates
     * {@link #runState}.  Enters DONE if no more steps remain.
     */
    private void nextStep() {
        stepIndex++;
        if (stepIndex >= steps.size()) {
            runState = RunState.DONE;
            return;
        }
        PathStep next = steps.get(stepIndex);
        if (next.path != null) {
            follower.followPath(next.path, true);
            runState = RunState.FOLLOWING_PATH;
        } else {
            // Null path (STARTING step): apply its action without travelling.
            runState = RunState.INIT_STEP;
        }
    }

    /** Drives the three-state machine; called every loop iteration. */
    private void updateStateMachine() {
        switch (runState) {

            case INIT_STEP:
                if (stepIndex < steps.size()) {
                    applyStepAction(steps.get(stepIndex));
                } else {
                    runState = RunState.DONE;
                }
                break;

            case FOLLOWING_PATH:
                if (!follower.isBusy()) {
                    runState = RunState.INIT_STEP;
                }
                break;

            case SHOOTING:
                if (System.currentTimeMillis() >= shootSequenceEndMs) {
                    nextStep();
                }
                break;

            case DONE:
                break;
        }
    }



    // ── OpMode lifecycle (final to prevent accidental override) ───────────────

    @Override
    public final void init() {
        config = getConfig();

        shootSequencer = new ShootSequencer();
        shootSequencer.init(hardwareMap);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(config.startPose);

        steps = (config.stepsFactory != null)
                ? config.stepsFactory.build(follower)
                : Collections.<PathStep>emptyList();

        stepIndex = 0;
        runState  = RunState.DONE; // start() activates the machine
    }

    @Override
    public void init_loop() {}

    @Override
    public final void start() {
        stepIndex          = 0;
        shootSequenceEndMs = 0;
        shootSequencer.setShooterRunningAfterShoot(true);
        double initialShooterTargetVelocity = config.shootTargetVelocity;
        if (!Double.isNaN(initialShooterTargetVelocity)) {
            shootSequencer.primeShooter(initialShooterTargetVelocity);
        }
        runState           = RunState.INIT_STEP;
    }

    @Override
    public final void loop() {
        follower.update();
        updateStateMachine();
        shootSequencer.loop(false);

        // ── Telemetry ──────────────────────────────────────────────────────────
        telemetry.addData("step",
                stepIndex + "/" + (steps.isEmpty() ? 0 : steps.size() - 1));
        telemetry.addData("state",     runState);
        telemetry.addData("hood pos", shootSequencer.getHoodPosition());
        telemetry.addData("target rpm", shootSequencer.getShooterTargetVelocity());
        telemetry.addData("current rpm", shootSequencer.getShooterVelocityRpm());
        telemetry.addData("shooter state", shootSequencer.getShootState());
        telemetry.addData("sequencer state", shootSequencer.getShootMode());
        telemetry.addData("spinup reason", shootSequencer.getSpinupEndReason());
        telemetry.addData("gate state", shootSequencer.isGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("spinner state", shootSequencer.getSpinnerState());
        telemetry.addData("intake state", shootSequencer.getIntakeState());
        if (!steps.isEmpty() && stepIndex < steps.size()) {
            telemetry.addData("pose type", steps.get(stepIndex).poseType);
        }
        if (runState == RunState.SHOOTING) {
            long remaining = Math.max(0, shootSequenceEndMs - System.currentTimeMillis());
            telemetry.addData("shoot remain ms", remaining);
        }
        telemetry.addData("x",       follower.getPose().getX());
        telemetry.addData("y",       follower.getPose().getY());
        telemetry.addData("heading", Math.toDegrees(follower.getPose().getHeading()));
        telemetry.update();
    }

    @Override
    public final void stop() {
        shootSequencer.setShooterRunningAfterShoot(false);
        shootSequencer.stopIntake();
        shootSequencer.stopShootingSequence();
    }
}
