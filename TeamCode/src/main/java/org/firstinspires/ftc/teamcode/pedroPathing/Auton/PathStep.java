package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.paths.PathChain;

/**
 * One step in an autonomous path sequence.
 *
 * <p>Each step pairs a {@link PathChain} (the path the robot follows to reach this
 * waypoint) with a {@link PoseType} (the role of the destination).  When the follower
 * finishes the path, {@link AutonBase} reads the pose type and applies the
 * corresponding subsystem action.
 *
 * <p>{@link PoseType#SCORING} steps carry their own shooter target velocity (RPM)
 * and hood position so that different scoring locations can use different settings.
 * Use {@link #scoring(PathChain, double, double)} to create them.  Non-scoring steps
 * use the two-argument constructor; their shoot fields are ignored.
 *
 * <p>The {@code path} field is {@code null} only for the
 * {@link PoseType#STARTING} step.
 */
public final class PathStep {

    /**
     * Type of the destination pose; drives the subsystem actions in {@link AutonBase}
     * when the robot arrives here.
     */
    public final PoseType  poseType;

    /**
     * Path the robot follows to reach this destination.
     * {@code null} for {@link PoseType#STARTING} (no movement required).
     */
    public final PathChain path;

    /**
     * Shooter target velocity (RPM) for this scoring step.
     * {@link Double#NaN} means "use the global default from {@link AutonConfig}".
     * Always {@link Double#NaN} for non-{@link PoseType#SCORING} steps.
     */
    public final double shootTargetVelocity;

    /**
     * Hood servo position (0.0 – 1.0) for this scoring step.
     * {@link Double#NaN} means "use the global default from {@link AutonConfig}".
     * Always {@link Double#NaN} for non-{@link PoseType#SCORING} steps.
     */
    public final double shootHoodPosition;

    /** General-purpose constructor (non-scoring steps). Shoot fields are set to NaN. */
    public PathStep(PoseType poseType, PathChain path) {
        this.poseType            = poseType;
        this.path                = path;
        this.shootTargetVelocity = Double.NaN;
        this.shootHoodPosition   = Double.NaN;
    }

    /** Full constructor. Use {@link #scoring} for a more readable call site. */
    public PathStep(PoseType poseType, PathChain path,
                    double shootTargetVelocity, double shootHoodPosition) {
        this.poseType            = poseType;
        this.path                = path;
        this.shootTargetVelocity = shootTargetVelocity;
        this.shootHoodPosition   = shootHoodPosition;
    }

    /**
     * Creates a {@link PoseType#SCORING} step with explicit shooter parameters.
     *
     * @param path                path to the scoring pose
     * @param shootTargetVelocity shooter target velocity in RPM
     * @param shootHoodPosition   hood servo position (0.0 – 1.0)
     */
    public static PathStep scoring(PathChain path,
                                   double shootTargetVelocity,
                                   double shootHoodPosition) {
        return new PathStep(PoseType.SCORING, path, shootTargetVelocity, shootHoodPosition);
    }
}
