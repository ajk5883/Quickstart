package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.Pose;

/**
 * Shared pose catalog for a specific autonomous routine and alliance combination.
 *
 * <p>A {@link PoseSet} groups the key field poses for one routine so they can be
 * passed around as a single object and referenced by {@link AutonConfig}.
 */
public final class AutonPoseConfig {

    private AutonPoseConfig() {}

    /**
     * A named collection of field poses for one autonomous routine / alliance.
     *
     * <p>At minimum {@code startPose} is required; add additional fields as
     * needed when a routine shares poses across multiple path files.
     */
    public static final class PoseSet {

        /** Starting pose passed to {@link com.pedropathing.follower.Follower#setStartingPose(Pose)}. */
        public final Pose startPose;

        public PoseSet(Pose startPose) {
            this.startPose = startPose;
        }
    }
}
