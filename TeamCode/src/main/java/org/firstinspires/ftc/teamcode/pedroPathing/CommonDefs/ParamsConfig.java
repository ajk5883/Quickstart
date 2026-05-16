package org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs;

import com.pedropathing.geometry.Pose;

/* Create an enum for RED / BLUE */
public enum Alliance {
    RED,
    BLUE
}

/**
 * Shared autonomous pose catalog.
 *
 * <p>Blue poses are the source-of-truth and red poses are computed by mirroring
 * across field center. Close-side variants are derived from long-side templates
 * by mirroring across the field's horizontal midline.
 */
public final class ParamsConfig {

    private ParamsConfig() {}

    public static final double FIELD_SIZE_IN = 144.0;

    // Ball-row poses first (blue source-of-truth).
    public static final Pose POSE_BALLS_ROW1_START_BLUE = new Pose(42.574, 109.417, Math.toRadians(180));
    public static final Pose POSE_BALLS_ROW1_END_BLUE = new Pose(24.381, 109.597, Math.toRadians(180));

    public static final Pose POSE_BALLS_ROW2_START_BLUE = new Pose(42.574, 103.417, Math.toRadians(180));
    public static final Pose POSE_BALLS_ROW2_END_BLUE = new Pose(24.381, 103.597, Math.toRadians(180));

    public static final Pose POSE_BALLS_ROW3_START_BLUE = new Pose(42.574, 97.417, Math.toRadians(180));
    public static final Pose POSE_BALLS_ROW3_END_BLUE = new Pose(24.381, 97.597, Math.toRadians(180));

    // Ball-row red poses mirrored from blue across the center vertical line.
    public static final Pose POSE_BALLS_ROW1_START_RED = mirrorAcrossVerticalMidline(POSE_BALLS_ROW1_START_BLUE);
    public static final Pose POSE_BALLS_ROW1_END_RED = mirrorAcrossVerticalMidline(POSE_BALLS_ROW1_END_BLUE);

    public static final Pose POSE_BALLS_ROW2_START_RED = mirrorAcrossVerticalMidline(POSE_BALLS_ROW2_START_BLUE);
    public static final Pose POSE_BALLS_ROW2_END_RED = mirrorAcrossVerticalMidline(POSE_BALLS_ROW2_END_BLUE);

    public static final Pose POSE_BALLS_ROW3_START_RED = mirrorAcrossVerticalMidline(POSE_BALLS_ROW3_START_BLUE);
    public static final Pose POSE_BALLS_ROW3_END_RED = mirrorAcrossVerticalMidline(POSE_BALLS_ROW3_END_BLUE);

    // Blue long-side source-of-truth poses.
    public static final Pose POSE_START_BLUE_LONG = new Pose(58.256, 136.516, Math.toRadians(90));
    public static final Pose POSE_SCORE_BLUE_LONG = new Pose(60.301, 129.391, Math.toRadians(62));
    public static final Pose POSE_PARK_BLUE_LONG = new Pose(33.620, 129.902);

    public static final Pose POSE_CORNER_PICKUP_START_BLUE_LONG = new Pose(17.109, 135.577);
    public static final Pose POSE_CORNER_PICKUP_BLUE_LONG = new Pose(6.399, 135.742);

    public static final Pose POSE_GATE_OPEN_PICKUP_BLUE_LONG = new Pose(42.574, 109.417, Math.toRadians(180));
    public static final Pose POSE_GATE_FEED_BLUE_LONG = new Pose(60.147, 129.491, Math.toRadians(62));

    // Red long-side poses mirrored from blue.
    public static final Pose POSE_START_RED_LONG = mirrorAcrossVerticalMidline(POSE_START_BLUE_LONG);
    public static final Pose POSE_SCORE_RED_LONG = mirrorAcrossVerticalMidline(POSE_SCORE_BLUE_LONG);
    public static final Pose POSE_PARK_RED_LONG = mirrorAcrossVerticalMidline(POSE_PARK_BLUE_LONG);

    public static final Pose POSE_CORNER_PICKUP_START_RED_LONG = mirrorAcrossVerticalMidline(POSE_CORNER_PICKUP_START_BLUE_LONG);
    public static final Pose POSE_CORNER_PICKUP_RED_LONG = mirrorAcrossVerticalMidline(POSE_CORNER_PICKUP_BLUE_LONG);

    public static final Pose POSE_GATE_OPEN_PICKUP_RED_LONG = mirrorAcrossVerticalMidline(POSE_GATE_OPEN_PICKUP_BLUE_LONG);
    public static final Pose POSE_GATE_FEED_RED_LONG = mirrorAcrossVerticalMidline(POSE_GATE_FEED_BLUE_LONG);

    // Blue close-side poses are explicit (not derived from long).
    public static final Pose POSE_START_BLUE_CLOSE = new Pose(58.256, 7.484, Math.toRadians(-90));
    public static final Pose POSE_SCORE_BLUE_CLOSE = new Pose(60.301, 14.609, Math.toRadians(-62));
    public static final Pose POSE_PARK_BLUE_CLOSE = new Pose(33.620, 14.098);

    public static final Pose POSE_CORNER_PICKUP_START_BLUE_CLOSE = new Pose(17.109, 8.423);
    public static final Pose POSE_CORNER_PICKUP_BLUE_CLOSE = new Pose(6.399, 8.258);

    public static final Pose POSE_GATE_OPEN_PICKUP_BLUE_CLOSE = new Pose(42.574, 34.583, Math.toRadians(180));
    public static final Pose POSE_GATE_FEED_BLUE_CLOSE = new Pose(60.147, 14.509, Math.toRadians(-62));

    // Red close-side poses mirrored from blue.
    public static final Pose POSE_START_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_START_BLUE_CLOSE);
    public static final Pose POSE_SCORE_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_SCORE_BLUE_CLOSE);
    public static final Pose POSE_PARK_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_PARK_BLUE_CLOSE);

    public static final Pose POSE_CORNER_PICKUP_START_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_CORNER_PICKUP_START_BLUE_CLOSE);
    public static final Pose POSE_CORNER_PICKUP_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_CORNER_PICKUP_BLUE_CLOSE);

    public static final Pose POSE_GATE_OPEN_PICKUP_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_GATE_OPEN_PICKUP_BLUE_CLOSE);
    public static final Pose POSE_GATE_FEED_RED_CLOSE = mirrorAcrossVerticalMidline(POSE_GATE_FEED_BLUE_CLOSE);

    public static Pose mirrorAcrossVerticalMidline(Pose bluePose) {
        double xRed = FIELD_SIZE_IN - bluePose.getX();
        double yRed = bluePose.getY();
        double headingRed = (bluePose.getHeading() + Math.PI) % (2 * Math.PI);
        return new Pose(xRed, yRed, headingRed);
    }

    // ── Auton shoot parameters ────────────────────────────────────────────────

    /** Timed gate-open window per shot (ms). Same for all shot positions. */
    public static final int    AUTON_SHOOT_DURATION_MS              = 900;

    /** Shooter target velocity (RPM) for far scoring positions (long-side). */
    public static final double AUTON_SHOOT_TARGET_VELOCITY_FAR      = 4800.0;
    /** Shooter target velocity (RPM) for close scoring positions (close-side). */
    public static final double AUTON_SHOOT_TARGET_VELOCITY_CLOSE    = 3500.0;

    /** Velocity tolerance (RPM) to consider shooter ready — far positions. */
    public static final double AUTON_SHOOT_VELOCITY_THRESHOLD_FAR   = 50.0;
    /** Velocity tolerance (RPM) to consider shooter ready — close positions. */
    public static final double AUTON_SHOOT_VELOCITY_THRESHOLD_CLOSE = 50.0;

    /** Total time to remain in SHOOTING state (ms). Same for all positions. */
    public static final long   AUTON_SHOOT_SEQUENCE_WAIT_MS         = 4500L;

    /** Hood servo position (0.0–1.0) for far scoring positions (long-side). */
    public static final double AUTON_SHOOT_HOOD_POSITION_FAR        = 0.35;
    /** Hood servo position (0.0–1.0) for close scoring positions (close-side). */
    public static final double AUTON_SHOOT_HOOD_POSITION_CLOSE      = 0.30;
}