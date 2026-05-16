package org.firstinspires.ftc.teamcode.pedroPathing.controller;

/**
 * Hardware-layer constants for every robot controller.
 *
 * <p>Centralises device names, servo/motor defaults, PIDF coefficients, and
 * physical motor constants that would otherwise be scattered as literals inside
 * individual controller classes.
 *
 * <p><b>Modify this file whenever physical hardware changes</b> — rewiring,
 * replacement motors, different gearboxes, or servo substitutions.
 *
 * <p>Gameplay / strategy constants (target velocities, shoot durations, etc.)
 * belong in {@link org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig}.
 * Teleop drive-feel tuning belongs in
 * {@link org.firstinspires.ftc.teamcode.pedroPathing.Teleop.TeleOpTuningConfig}.
 */
public final class ControllerParams {

    private ControllerParams() {}

    // ── Hardware-map device names ─────────────────────────────────────────────
    // These strings must exactly match the names entered in the robot configuration
    // on the Driver Hub / RC app.

    public static final String HW_SHOOTER_MOTOR1 = "launch1";
    public static final String HW_SHOOTER_MOTOR2 = "launch2";
    public static final String HW_INTAKE_MOTOR   = "intakefb";
    public static final String HW_SPINNER_MOTOR  = "bslr";
    public static final String HW_GATE_SERVO     = "gate";
    public static final String HW_HOOD_SERVO     = "hood";
    public static final String HW_LIMELIGHT      = "limelight";

    // ── Shooter motor physics ─────────────────────────────────────────────────

    /** Encoder ticks per revolution for the shooter motor (REV HD Hex Motor). */
    public static final double SHOOTER_TICKS_PER_REV = 28.0;

    /**
     * Default RPM tolerance: how close to target RPM counts as "shooter ready".
     * Represents mechanical precision of the motor controller, not strategy.
     */
    public static final double SHOOTER_VELOCITY_THRESHOLD_RPM = 50.0;

    /** Velocity PIDF coefficients applied to the shooter motor in RUN_USING_ENCODER mode. */
    public static final double SHOOTER_PIDF_P = 14.0;
    public static final double SHOOTER_PIDF_I = 0.0;
    public static final double SHOOTER_PIDF_D = 0.4;
    public static final double SHOOTER_PIDF_F = 13.6;

    // ── Gate servo positions ──────────────────────────────────────────────────

    /** Servo position (0.0–1.0) when the gate is fully open (ball released). */
    public static final double GATE_OPEN_POSITION   = 0.1;
    /** Servo position (0.0–1.0) when the gate is fully closed (ball retained). */
    public static final double GATE_CLOSED_POSITION = 0.4;

    // ── Hood servo ────────────────────────────────────────────────────────────

    /** Minimum servo position for the hood (lowest angle). */
    public static final double HOOD_MIN_POSITION     = 0.1;
    /** Maximum servo position for the hood (highest angle). */
    public static final double HOOD_MAX_POSITION     = 1.0;
    /** Initial hood servo position applied at init (fully retracted). */
    public static final double HOOD_DEFAULT_POSITION = 0.1;

    // ── Intake motor ──────────────────────────────────────────────────────────

    /** Full power in the intake (ball-in) direction. Negative due to motor mounting. */
    public static final double INTAKE_POWER_FORWARD = -1.0;
    /** Full power in the eject (ball-out) direction. */
    public static final double INTAKE_POWER_REVERSE =  1.0;

    // ── Ball spinner motor ────────────────────────────────────────────────────

    /** Motor power when the spinner is active. Negative due to motor mounting direction. */
    public static final double SPINNER_POWER_ON = -1.0;

    // ── Camera ────────────────────────────────────────────────────────────────

    /** Limelight result poll rate in Hz. */
    public static final int    CAMERA_POLL_RATE_HZ       = 60;

    /**
     * Degrees to subtract from the raw Limelight yaw to align with field heading.
     * Accounts for the 90° rotational offset between the Limelight frame and the
     * Pedro Pathing field frame. Applied uniformly for all yaw values.
     */
    public static final double CAMERA_HEADING_OFFSET_DEG = 90.0;
}
