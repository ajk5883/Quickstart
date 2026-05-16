package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;

/**
 * High-level shoot sequencer that holds references to the shooter, hood, gate and feeder spinner.
 * This class provides simple init and convenience control methods; it intentionally keeps logic
 * minimal so callers can implement timing/state machines in their OpModes.
 */
public class ShootSequencer {
    public enum ShootState {
        IDLE,
        SPINUP,
        SHOOTING,
        STOPPING
    }

    public enum ShootMode {
        NONE,
        TIMED,
        ACTIVE_FLAG
    }

    public enum SpinnerState {
        OFF,
        ON
    }

    public enum IntakeState {
        OFF,
        FORWARD,
        REVERSE
    }

    private GateController gate;
    private HoodController hood;
    private Shooter shooter;
    private BallSpinnerController spinner;
    private IntakeController intake;

    // Sequencer defaults — sourced from config files, not hard-coded literals
    private static final int    DEFAULT_SPINUP_TIMEOUT_MS          = ParamsConfig.SHOOTER_SPINUP_TIMEOUT_MS;
    private static final double DEFAULT_SHOOTER_TARGET_VELOCITY    = ParamsConfig.SHOOTER_DEFAULT_TARGET_RPM;
    private static final double DEFAULT_SHOOTER_VELOCITY_THRESHOLD = ParamsConfig.SHOOTER_DEFAULT_VELOCITY_THRESHOLD;
    private static final double DEFAULT_HOOD_POSITION              = ControllerParams.HOOD_DEFAULT_POSITION;

    private double targetHoodPosition = DEFAULT_HOOD_POSITION;
    
    private int spinupTimeoutMs;
    private double shooterTargetVelocity;
    private double shooterVelocityThreshold;


    public ShootSequencer() {
        gate = new GateController();
        hood = new HoodController();
        shooter = new Shooter();
        spinner = new BallSpinnerController();
        intake = new IntakeController();
    }

    public void setHoodPosition(double position) {
        targetHoodPosition = Math.max(0.0, Math.min(1.0, position));
        hood.setPosition(targetHoodPosition);
    }

    public double getHoodPosition() {
        return hood.getPosition();
    }

    /**
     * Initialize all sub-controllers with device names and parameters.
     */
    public void init(HardwareMap hardwareMap) {
        shooter.init(hardwareMap);
        hood.init(hardwareMap);
        gate.init(hardwareMap);
        spinner.init(hardwareMap);
        intake.init(hardwareMap);
        setSpinupTimeoutMs(DEFAULT_SPINUP_TIMEOUT_MS);
        setShooterTargetVelocity(DEFAULT_SHOOTER_TARGET_VELOCITY);
        setShooterVelocityThreshold(DEFAULT_SHOOTER_VELOCITY_THRESHOLD);

        // Ensure everything starts in a safe default state
        gate.closeGate();
        turnOffSpinner();
        shooter.stopShooter();
        hood.setPosition(DEFAULT_HOOD_POSITION);
        intake.turnOffIntake();


    }

    // ---- Sub-controller accessors removed in favour of the facade below ----

    // Gate
    public void openGate() { gate.openGate(); }
    public void closeGate() { gate.closeGate(); }
    public void setGatePosition(double position) { gate.setPosition(position); }
    public double getGatePosition() { return gate.getPosition(); }

    // Shooter (direct, bypasses the sequencer state machine – for testing / tuning)
    public void startShooterDirectly(double rpm) { shooter.startShooter(rpm); }
    public void stopShooterDirectly() { shooter.stopShooter(); }
    public double getShooterVelocityRpm() { return shooter.getAverageVelocityRpm(); }
    public double getShooterTargetVelocity() { return shooterTargetVelocity; }
    public boolean isShooterVelocityWithinThreshold() { return shooter.isVelocityWithinThreshold(); }

    // Spinner (direct)
    public void turnOnSpinner() { spinner.turnOn(); spinnerState = SpinnerState.ON; }
    public void turnOffSpinner() { spinner.turnOff(); spinnerState = SpinnerState.OFF; }
    public SpinnerState getSpinnerState() { return spinnerState; }
    public IntakeState getIntakeState() { return intakeState; }
    public ShootState getShootState() { return shootState; }
    public ShootMode getShootMode() { return shootMode; }
    public boolean isTimedMode() { return shootMode == ShootMode.TIMED; }
    public boolean isGateOpen() {
        double position = gate.getPosition();
        return Math.abs(position - ControllerParams.GATE_OPEN_POSITION) <= Math.abs(position - ControllerParams.GATE_CLOSED_POSITION);
    }
    public boolean isShootingSequenceActive() { return shootingSequenceActive; }
    private ShootState shootState = ShootState.IDLE;
    private ShootMode shootMode = ShootMode.NONE;
    private SpinnerState spinnerState = SpinnerState.OFF;
    private IntakeState intakeState = IntakeState.OFF;
    private boolean shootingSequenceActive = false;
    private boolean intakeActive = false;
    private boolean stopRequested = false;
    private boolean keepShooterRunningAfterShoot = false;
    private long shootStateStartMs = 0;
    private long shootingDurationMs = 0;



    /**
     * Call repeatedly from your OpMode loop to advance the sequencer's state machine.
     */
    public void loop(boolean bshooterActiveRequested) {
        if (!shootingSequenceActive) {
            return;
        }

        long now = System.currentTimeMillis();

        switch (shootState) {
            case IDLE:
                // Idle state does nothing; active sequences always start at SPINUP.
                break;

            case SPINUP:
                if (stopRequested) {
                    shootState = ShootState.STOPPING;
                    break;
                }

                hood.setPosition(targetHoodPosition);
                shooter.startShooter(shooterTargetVelocity);
                
                if (shooter.isVelocityWithinThreshold() || now - shootStateStartMs >= spinupTimeoutMs) {
                    gate.openGate();
                    shootState = ShootState.SHOOTING;
                    shootStateStartMs = now;
                }
                break;

            case SHOOTING:
                gate.openGate();
                turnOnSpinner();
                
                if (stopRequested) {
                    shootState = ShootState.STOPPING;
                } else if (shootMode == ShootMode.TIMED) {
                    if (now - shootStateStartMs >= shootingDurationMs) {
                        shootState = ShootState.STOPPING;
                    }
                } else if (shootMode == ShootMode.ACTIVE_FLAG) {
                    if (!bshooterActiveRequested) {
                        shootState = ShootState.STOPPING;
                    }
                }
                break;

            case STOPPING:
                gate.closeGate();
                if (!intakeActive) {
                    turnOffSpinner();
                }
                stopRequested = false; // reset stop request for next time

                if (!keepShooterRunningAfterShoot) {
                    shooter.stopShooter();
                }
                
                shootingSequenceActive = false;                
                shootMode = ShootMode.NONE;
                shootState = ShootState.IDLE;
                break;
        }
    }

    /**
     * Start a timed shooting sequence. Intended for autonomous use.
     */
    public void startShootingSequence(int shootingDurationMs, double targetVelocity, double velocityThreshold) {
        this.shootingDurationMs = Math.max(0, shootingDurationMs);
        this.shootMode = ShootMode.TIMED;
        this.shooterTargetVelocity = targetVelocity;
        this.shooterVelocityThreshold = velocityThreshold;
        this.shooter.setVelocityThreshold(velocityThreshold);
        this.shootingSequenceActive = true;
        this.shootState = ShootState.SPINUP;
        this.stopRequested = false;
        this.shootStateStartMs = System.currentTimeMillis();
        
        this.gate.closeGate();
        if (!this.intakeActive) {
            this.turnOffSpinner();
        }
    }

    /**
     * Start a shooting sequence that stays active while `shooterActive` remains true.
     * Intended for teleop use with a button press.
     */
    public void startShootingSequence(double targetVelocity, double velocityThreshold) {
        this.shootMode = ShootMode.ACTIVE_FLAG;
        this.shooterTargetVelocity = targetVelocity;
        this.shooterVelocityThreshold = velocityThreshold;
        this.shooter.setVelocityThreshold(velocityThreshold);
        this.shootingSequenceActive = true;
        this.shootState = ShootState.SPINUP;
        this.stopRequested = false;
        this.shootStateStartMs = System.currentTimeMillis();
        
        this.gate.closeGate();
        if (!this.intakeActive) {
            this.turnOffSpinner();
        }
    }


    /**
     * Stop any active shooting sequence and ensure all subsystems are closed/stopped.
     */
    public void stopShootingSequence() {
        this.stopRequested = true;
        
        if (!shootingSequenceActive) {
            // If the loop isn't active to process the state machine, force shutdown now
            gate.closeGate();
            if (!intakeActive) {
                turnOffSpinner();
            }
            if (!keepShooterRunningAfterShoot) {
                shooter.stopShooter();
            }
            this.stopRequested = false;
        }
    }

    /**
     * Intake control: turns the feeder/spinner on (intake) or off.
     * When intake is on, the spinner is forced on regardless of shooter state.
     */
    public void startIntake() {
        intakeActive = true;
        turnOnSpinner();
        intake.turnOnIntake();
        intakeState = IntakeState.FORWARD;
    }

    public void stopIntake() {
        intakeActive = false;
        intake.turnOffIntake();
        intakeState = IntakeState.OFF;
        if(shootState != ShootState.SHOOTING) {
            turnOffSpinner();
        }
    }

    /**
     * Run intake in reverse (eject). Clears intakeActive so spinner is not forced on.
     */
    public void startIntakeReverse() {
        intakeActive = false;
        intake.turnOnIntakeReverse();
        intakeState = IntakeState.REVERSE;
    }

    /**
     * Controls whether the feeder/spinner should continue running after the shooting phase ends.
     */
    public void setShooterRunningAfterShoot(boolean keep) {
        this.keepShooterRunningAfterShoot = keep;

    }

    // Optional setters for tuning
    public void setSpinupTimeoutMs(int ms) { this.spinupTimeoutMs = ms; }
    public void setShooterTargetVelocity(double v) { this.shooterTargetVelocity = v; }
    public void setShooterVelocityThreshold(double t) {
        this.shooterVelocityThreshold = t;
        this.shooter.setVelocityThreshold(t);
    }
}
