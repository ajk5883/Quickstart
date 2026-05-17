package org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs;

import static org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.normalizeRadians;

import com.pedropathing.geometry.Pose;

/* Create an enum for RED / BLUE */


/**
 * Shared autonomous pose catalog.
 *
 * <p>Blue poses are the source-of-truth and red poses are computed by mirroring
 * across field center. Close-side variants are derived from long-side templates
 * by mirroring across the field's horizontal midline.
 */
public final class ParamsConfig {

    public enum Alliance {
        RED,
        BLUE
    }

    private ParamsConfig() {}

    public static final double FIELD_SIZE_IN = 144.0;
    public static final double ROBOT_WHEELBASE_WIDTH_IN = 16.0; // Used for path offsetting to prevent collisions with field elements
    public static final double ROBOT_WHEELBASEHEIGHT_IN = 7.8;

    public static final double ROBOT_WHEELBASE_HALFWIDTH = ROBOT_WHEELBASE_WIDTH_IN / 2; // Used for path offsetting to prevent collisions with field elements
    public static final double ROBOT_WHEELBASE_HALFHEIGHT = ROBOT_WHEELBASEHEIGHT_IN / 2; // Used for path offsetting to prevent collisions with field elements

    public static final double Adjustment_Long_Y=8.0; // Used to adjust the Y values of the long paths to better align with the field elements
    public static double BLUE_ROW1_Y = 35.0 - Adjustment_Long_Y;
    public static double BLUE_ROW_X_Start = 44.0;
    public static double BLUE_ROW_X_End = 22.0;

    // Ball-row poses first (blue source-of-truth).
    public static final Pose POSE_BALLS_ROW1_START_BLUE = new Pose(BLUE_ROW_X_Start, BLUE_ROW1_Y, Math.toRadians(180));
    public static final Pose POSE_BALLS_ROW1_END_BLUE = new Pose(BLUE_ROW_X_End, BLUE_ROW1_Y, Math.toRadians(180));

    public static final Pose POSE_BALLS_ROW2_START_BLUE = new Pose(BLUE_ROW_X_Start, BLUE_ROW1_Y+24, Math.toRadians(180));
    public static final Pose POSE_BALLS_ROW2_END_BLUE = new Pose(BLUE_ROW_X_End, BLUE_ROW1_Y+24, Math.toRadians(180));

    public static final Pose POSE_BALLS_ROW3_START_BLUE = new Pose(BLUE_ROW_X_Start, BLUE_ROW1_Y+48, Math.toRadians(180));
    public static final Pose POSE_BALLS_ROW3_END_BLUE = new Pose(BLUE_ROW_X_End, BLUE_ROW1_Y+48, Math.toRadians(180));

    // Ball-row red poses mirrored from blue across the center vertical line.
    public static final Pose POSE_BALLS_ROW1_START_RED = new Pose(FIELD_SIZE_IN - BLUE_ROW_X_Start, BLUE_ROW1_Y, Math.toRadians(0));
    public static final Pose POSE_BALLS_ROW1_END_RED = new Pose(FIELD_SIZE_IN - BLUE_ROW_X_End, BLUE_ROW1_Y, Math.toRadians(0));

    public static final Pose POSE_BALLS_ROW2_START_RED = new Pose(FIELD_SIZE_IN - BLUE_ROW_X_Start, BLUE_ROW1_Y+24, Math.toRadians(0));
    public static final Pose POSE_BALLS_ROW2_END_RED = new Pose(FIELD_SIZE_IN - BLUE_ROW_X_End, BLUE_ROW1_Y+24, Math.toRadians(0));

    public static final Pose POSE_BALLS_ROW3_START_RED = new Pose(FIELD_SIZE_IN - BLUE_ROW_X_Start, BLUE_ROW1_Y+48, Math.toRadians(0));
    public static final Pose POSE_BALLS_ROW3_END_RED = new Pose(FIELD_SIZE_IN - BLUE_ROW_X_End, BLUE_ROW1_Y+48, Math.toRadians(0));

    // Blue long-side source-of-truth poses.
    public static final Pose POSE_START_BLUE_LONG = new Pose(48+ROBOT_WHEELBASE_HALFWIDTH, 11, Math.toRadians(270));
    public static final Pose POSE_SCORE_BLUE_LONG = new Pose(58, 12, Math.toRadians(298));
    public static final Pose POSE_PARK_BLUE_LONG = new Pose(54, 54, Math.toRadians(180));

    public static final Pose POSE_CORNER_PICKUP_START_BLUE_LONG = new Pose(10+ ROBOT_WHEELBASE_HALFWIDTH,0 + ROBOT_WHEELBASE_HALFHEIGHT, Math.toRadians(180));
    public static final Pose POSE_CORNER_PICKUP_BLUE_LONG = new Pose(5+ ROBOT_WHEELBASE_HALFWIDTH,0 + ROBOT_WHEELBASE_HALFHEIGHT,Math.toRadians(180));

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
    public static final Pose POSE_START_BLUE_CLOSE = new Pose(58.256, 7.484, Math.toRadians(270));
    public static final Pose POSE_SCORE_BLUE_CLOSE = new Pose(60.301, 14.609, Math.toRadians(298));
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
        double headingRed = normalizeRadians(Math.PI - bluePose.getHeading());
        return new Pose(xRed, yRed, headingRed);
    }

    // ── Auton shoot parameters ────────────────────────────────────────────────

    /** Timed gate-open window per shot (ms). Same for all shot positions. */
    public static final int    AUTON_SHOOT_DURATION_MS              = 1000;

    /** Shooter target velocity (RPM) for far scoring positions (long-side). */
    public static final double AUTON_SHOOT_TARGET_VELOCITY_FAR      = 4800.0;
    /** Shooter target velocity (RPM) for close scoring positions (close-side). */
    public static final double AUTON_SHOOT_TARGET_VELOCITY_CLOSE    = 3500.0;

    /** Velocity tolerance (RPM) to consider shooter ready — far positions. */
    public static final double AUTON_SHOOT_VELOCITY_THRESHOLD_FAR   = 100.0;
    /** Velocity tolerance (RPM) to consider shooter ready — close positions. */
    public static final double AUTON_SHOOT_VELOCITY_THRESHOLD_CLOSE = 100.0;

    /** Total time to remain in SHOOTING state (ms). Same for all positions. */
    public static final long   AUTON_SHOOT_SEQUENCE_WAIT_MS         = 4500L;

    /** Hood servo position (0.0–1.0) for far scoring positions (long-side). */
    public static final double AUTON_SHOOT_HOOD_POSITION_FAR        = 0.5;
    /** Hood servo position (0.0–1.0) for close scoring positions (close-side). */
    public static final double AUTON_SHOOT_HOOD_POSITION_CLOSE      = 0.8;

    /** Auton-only aggressive shooter PIDF values used to spin up faster at the start of a run. */
    public static final double AUTON_SHOOT_PIDF_P_AGGRESSIVE        = 16.0;
    public static final double AUTON_SHOOT_PIDF_I_AGGRESSIVE        = 0.0;
    public static final double AUTON_SHOOT_PIDF_D_AGGRESSIVE        = 0.45;
    public static final double AUTON_SHOOT_PIDF_F_AGGRESSIVE        = 14.5;

    /** Teleop long-shot shooter target velocity (RPM). */
    public static final double TELEOP_SHOOT_TARGET_VELOCITY_LONG    = AUTON_SHOOT_TARGET_VELOCITY_FAR;
    /** Teleop close-shot shooter target velocity (RPM). */
    public static final double TELEOP_SHOOT_TARGET_VELOCITY_CLOSE   = AUTON_SHOOT_TARGET_VELOCITY_CLOSE;

    /** Teleop long-shot hood position (0.0–1.0). */
    public static final double TELEOP_SHOOT_HOOD_POSITION_LONG     = AUTON_SHOOT_HOOD_POSITION_FAR;
    /** Teleop close-shot hood position (0.0–1.0). */
    public static final double TELEOP_SHOOT_HOOD_POSITION_CLOSE    = AUTON_SHOOT_HOOD_POSITION_CLOSE;

    // ── ShootSequencer strategy defaults ─────────────────────────────────────────

    /**
     * Maximum time (ms) allowed for the shooter to spin up before the gate opens
     * regardless of whether the target velocity has been reached.
     */
    public static final int    SHOOTER_SPINUP_TIMEOUT_MS          = 2000;

    /**
     * Default shooter target RPM used by ShootSequencer when no explicit velocity is
     * provided. Intentionally aligned with the far-side auton value to prevent
     * unexpected behaviour. (Previously was 6000 RPM — inconsistent with tuned values.)
     */
    public static final double SHOOTER_DEFAULT_TARGET_RPM         = AUTON_SHOOT_TARGET_VELOCITY_FAR;

    /**
     * Default RPM tolerance for the ShootSequencer when no explicit threshold is
     * supplied at the strategy level.
     */
    public static final double SHOOTER_DEFAULT_VELOCITY_THRESHOLD = 100.0;
}