package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.ControllerParams;
import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;

/**
 * Shooter controller supporting single- or dual-motor configurations.
 * Defaults to a single motor named "launch1". Motors are reversed by default
 * (common for mirrored shooter motor installations) and can be configured
 * to run using encoders for velocity control.
 */
public class Shooter {
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;

    private String motor1Name = ControllerParams.HW_SHOOTER_MOTOR1;
    private String motor2Name = ControllerParams.HW_SHOOTER_MOTOR2;

    private boolean hasSecondMotor = false;
    private boolean motorsReversed = true;
    private boolean useEncoders = true;

    // targetVelocity is expressed in RPM (human-friendly). Internally we convert
    // to ticks/sec for the DcMotorEx velocity calls.
    private double targetVelocity = 0.0; // RPM
    private double velocityThreshold = ParamsConfig.SHOOTER_DEFAULT_VELOCITY_THRESHOLD; // RPM

    private static final double TICKS_PER_REV = ControllerParams.SHOOTER_TICKS_PER_REV;

    public Shooter() {}

    // --- Initialization ---
    public void init(HardwareMap hardwareMap) {

        shooterMotor1 = hardwareMap.get(DcMotorEx.class, this.motor1Name);
        shooterMotor1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        if (motorsReversed) shooterMotor1.setDirection(DcMotorSimple.Direction.REVERSE);
        if (useEncoders) shooterMotor1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        if (hasSecondMotor) {
            shooterMotor2 = hardwareMap.get(DcMotorEx.class, this.motor2Name);
            shooterMotor2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            if (motorsReversed) shooterMotor2.setDirection(DcMotorSimple.Direction.REVERSE);
            if (useEncoders) shooterMotor2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        } 
        else {
            shooterMotor2 = null;
        }

        // setVelocityPIDF(
        //         ControllerParams.SHOOTER_PIDF_P,
        //         ControllerParams.SHOOTER_PIDF_I,
        //         ControllerParams.SHOOTER_PIDF_D,
        //         ControllerParams.SHOOTER_PIDF_F);
    }


  

    // --- Setters ---
    public void setTargetVelocity(double targetVelocity) { this.targetVelocity = targetVelocity; }
    /**
     * Set velocity threshold in RPM (how close to targetRPM counts as "ready").
     */
    public void setVelocityThreshold(double velocityThreshold) { this.velocityThreshold = velocityThreshold; }

    /** Set motor velocity PIDF coefficients (applied when encoders are used). */
    public void setVelocityPIDF(double p, double i, double d, double f) {
        PIDFCoefficients coeffs = new PIDFCoefficients(p, i, d, f);
        if (shooterMotor1 != null && useEncoders) shooterMotor1.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
        if (shooterMotor2 != null && hasSecondMotor && useEncoders) shooterMotor2.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, coeffs);
    }

    public static double rpmToTicksPerSecond(double rpm) {
        return (rpm / 60.0) * TICKS_PER_REV;
    }

    public static double ticksPerSecondToRpm(double ticksPerSecond) {
        return (ticksPerSecond / TICKS_PER_REV) * 60.0;
    }

    // --- Control ---
    // startShooter expects targetVelocity in RPM; converts to ticks/sec for motor controllers
    public void startShooter(double targetVelocity) {
        setTargetVelocity(targetVelocity);
        double targetTicksPerSec = rpmToTicksPerSecond(targetVelocity);
        if (shooterMotor1 != null) shooterMotor1.setVelocity(targetTicksPerSec);
        if (hasSecondMotor && shooterMotor2 != null) shooterMotor2.setVelocity(targetTicksPerSec);
    }

    public void stopShooter() {
        if (shooterMotor1 != null) shooterMotor1.setVelocity(0);
        if (hasSecondMotor && shooterMotor2 != null) shooterMotor2.setVelocity(0);
    }

    // --- Telemetry / status ---
    public boolean isVelocityWithinThreshold() {
        if (shooterMotor1 == null) return false;
        double targetTicksPerSec = rpmToTicksPerSecond(targetVelocity);
        double thresholdTicks = rpmToTicksPerSecond(velocityThreshold);
        double motor1Error = Math.abs(shooterMotor1.getVelocity() - targetTicksPerSec);
        if (!hasSecondMotor || shooterMotor2 == null) {
            return motor1Error <= thresholdTicks;
        }
        double motor2Error = Math.abs(shooterMotor2.getVelocity() - targetTicksPerSec);
        return motor1Error <= thresholdTicks && motor2Error <= thresholdTicks;
    }

    public double getAverageVelocity() {
        if (shooterMotor1 == null) return 0.0;
        if (!hasSecondMotor || shooterMotor2 == null) return Math.abs(shooterMotor1.getVelocity());
        return (Math.abs(shooterMotor1.getVelocity()) + Math.abs(shooterMotor2.getVelocity())) / 2.0;
    }

    public double getTargetVelocity() { return targetVelocity; }

    /** Return average velocity in RPM */
    public double getAverageVelocityRpm() {
        return ticksPerSecondToRpm(getAverageVelocity());
    }
}
