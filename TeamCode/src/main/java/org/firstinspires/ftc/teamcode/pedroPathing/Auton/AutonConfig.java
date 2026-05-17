package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;

import java.util.List;

/**
 * Immutable configuration for one autonomous routine.
 *
 * <p>Rather than encoding a fixed pose-field schema, {@code AutonConfig} accepts a
 * {@link StepsFactory} — a single callback that receives the {@link Follower} (required
 * to build {@link com.pedropathing.paths.PathChain}s) and returns the ordered list of
 * {@link PathStep}s to execute.  The factory is called once inside
 * {@code AutonBase.init()} after the follower is created.
 *
 * <p>Defining poses and building path chains inside the factory (in the concrete
 * subclass of {@code AutonBase}) keeps each routine's geometry entirely self-contained
 * while allowing {@code AutonBase} to drive any step sequence — long-side, close-side,
 * or any future layout — without knowing the field geometry.
 *
 * <p><b>Typical usage in a subclass</b>
 * <pre>
 *   {@literal @}Override
 *   protected AutonConfig getConfig() {
 *       final Pose start = new Pose(...);
 *       final Pose score = new Pose(...);
 *       // ... more poses ...
 *
 *       return AutonConfig.withDefaults(start, follower -> {
 *           PathChain toScore = follower.pathBuilder()
 *                   .addPath(new BezierLine(start, score))
 *                   .setLinearHeadingInterpolation(start.getHeading(), score.getHeading())
 *                   .build();
 *           List&lt;PathStep&gt; steps = new ArrayList&lt;&gt;();
 *           steps.add(new PathStep(PoseType.STARTING,  null));
 *           steps.add(new PathStep(PoseType.SCORING,   toScore));
 *           // ... more steps ...
 *           steps.add(new PathStep(PoseType.PARKING,   toPark));
 *           return steps;
 *       });
 *   }
 * </pre>
 *
 * Shoot parameters
 * ────────────────
 *   shootDurationMs         – Timed gate-open window per shot (ms).
 *   shootTargetVelocity     – Shooter target velocity (RPM).
 *   shootVelocityThreshold  – Velocity tolerance to consider shooter "ready" (RPM).
 *   shootSequenceWaitMs     – Total time to remain in SHOOTING state (ms).
 *                             Must be >= shootDurationMs; covers spin-up + shooting margin.
 *   shootHoodPosition       – Hood servo position (0.0 – 1.0) applied before every shot.
 */
public final class AutonConfig {

    // ── StepsFactory ──────────────────────────────────────────────────────────

    /**
     * Builds the ordered list of {@link PathStep}s once a {@link Follower} is available.
     *
     * <p>Implementations should declare all {@link Pose} objects and build all
     * {@link com.pedropathing.paths.PathChain}s inside this method so that the path
     * geometry is co-located with the rest of the routine's configuration.
     */
    @FunctionalInterface
    public interface StepsFactory {
        List<PathStep> build(Follower follower);
    }

    // ── Fields ────────────────────────────────────────────────────────────────

    /** Starting pose passed to {@link Follower#setStartingPose(Pose)}. */
    public final Pose         startPose;

    /** Builds the step sequence; called once during {@code AutonBase.init()}. */
    public final StepsFactory stepsFactory;

    // Shoot parameters
    public final int    shootDurationMs;
    public final double shootTargetVelocity;
    public final double shootVelocityThreshold;
    public final long   shootSequenceWaitMs;
    public final double shootHoodPosition;

    // ── Full constructor ──────────────────────────────────────────────────────

    public AutonConfig(
            Pose         startPose,
            StepsFactory stepsFactory,
            int          shootDurationMs,
            double       shootTargetVelocity,
            double       shootVelocityThreshold,
            long         shootSequenceWaitMs,
            double       shootHoodPosition) {

        this.startPose              = startPose;
        this.stepsFactory           = stepsFactory;
        this.shootDurationMs        = shootDurationMs;
        this.shootTargetVelocity    = shootTargetVelocity;
        this.shootVelocityThreshold = shootVelocityThreshold;
        this.shootSequenceWaitMs    = shootSequenceWaitMs;
        this.shootHoodPosition      = shootHoodPosition;
    }

    // ── Convenience factory ───────────────────────────────────────────────────

    /**
     * Creates an {@code AutonConfig} using far-side (long-side) shoot defaults.
     * Use when tuning is not required and per-step overrides handle the rest.
     */
    // public static AutonConfig withDefaults(Pose startPose, StepsFactory stepsFactory) {
    //     return new AutonConfig(
    //             startPose,
    //             null,
    //             stepsFactory,
    //             ParamsConfig.AUTON_SHOOT_DURATION_MS,
    //             ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_FAR,
    //             ParamsConfig.AUTON_SHOOT_VELOCITY_THRESHOLD_FAR,
    //             ParamsConfig.AUTON_SHOOT_SEQUENCE_WAIT_MS,
    //             ParamsConfig.AUTON_SHOOT_HOOD_POSITION_FAR);
    // }

    /**
     * Creates an {@code AutonConfig} using far-side (long-side) shoot defaults.
     */
    public static AutonConfig withFarDefaults(Pose startPose, StepsFactory stepsFactory) {
        return new AutonConfig(
                startPose,
                stepsFactory,
                ParamsConfig.AUTON_SHOOT_DURATION_MS,
                ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_FAR,
                ParamsConfig.AUTON_SHOOT_VELOCITY_THRESHOLD_FAR,
                ParamsConfig.AUTON_SHOOT_SEQUENCE_WAIT_MS,
                ParamsConfig.AUTON_SHOOT_HOOD_POSITION_FAR);
    }

    /**
     * Creates an {@code AutonConfig} using close-side shoot defaults.
     */
    public static AutonConfig withCloseDefaults(Pose startPose, StepsFactory stepsFactory) {
        return new AutonConfig(
                startPose,
                stepsFactory,
                ParamsConfig.AUTON_SHOOT_DURATION_MS,
                ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_CLOSE,
                ParamsConfig.AUTON_SHOOT_VELOCITY_THRESHOLD_CLOSE,
                ParamsConfig.AUTON_SHOOT_SEQUENCE_WAIT_MS,
                ParamsConfig.AUTON_SHOOT_HOOD_POSITION_CLOSE);
    }
}
