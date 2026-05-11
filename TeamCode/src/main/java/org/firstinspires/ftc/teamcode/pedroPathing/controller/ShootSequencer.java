package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * High-level shoot sequencer that holds references to the shooter, hood, gate and feeder spinner.
 * This class provides simple init and convenience control methods; it intentionally keeps logic
 * minimal so callers can implement timing/state machines in their OpModes.
 */
public class ShootSequencer {
    private GateController gate;
    private HoodController hood;
    private Shooter shooter;
    private BallSpinnerController spinner;

    // Configurable timing/velocity parameters (can be changed by caller)
    private int Default_spinupTimeoutMs = 3000;
    private double Default_shooterTargetVelocity = 3000.0;
    private double Default_shooterVelocityThreshold = 50.0;
    private int Default_hoodposition_Angle = 0;

    private int Target_Hoodposition_Angle = 0;
    
    private int spinupTimeoutMs;
    private double shooterTargetVelocity;
    private double shooterVelocityThreshold;


    public ShootSequencer() {
        gate = new GateController();
        hood = new HoodController();
        shooter = new Shooter();
        spinner = new BallSpinnerController();
    }

    public void Set_Hoodposition_Angle(int angle) {
        Target_Hoodposition_Angle = angle;
    }

    /**
     * Initialize all sub-controllers with device names and parameters.
     */
    public void init(HardwareMap hardwareMap,
                     String shooterMotor1Name, String shooterMotor2Name,
                     String hoodServoName, double hoodMinPos, double hoodMaxPos,
                     String gateServoName, double gateOpenPos, double gateClosedPos,
                     String spinnerMotorName) {
        shooter.init(hardwareMap, shooterMotor1Name, shooterMotor2Name);
        hood.init(hardwareMap, hoodServoName, hoodMinPos, hoodMaxPos);
        gate.init(hardwareMap, gateServoName, gateOpenPos, gateClosedPos);
        spinner.init(hardwareMap, spinnerMotorName);

        setSpinupTimeoutMs(Default_spinupTimeoutMs);
        setShooterTargetVelocity(Default_shooterTargetVelocity);
        setShooterVelocityThreshold(Default_shooterVelocityThreshold); 
        
        

        // Ensure everything starts in a safe default state
        gate.closeGate();
        spinner.turnOff();
        shooter.stopShooter();
        hood.setAngle(Default_hoodposition_Angle);


    }

    // Accessors
    public GateController getGate() { return gate; }
    public HoodController getHood() { return hood; }
    public Shooter getShooter() { return shooter; }
    public BallSpinnerController getSpinner() { return spinner; }

    private enum ShootState {
        IDLE,
        SPINUP,
        SHOOTING,
        STOPPING
    }

    private enum ShootMode {
        NONE,
        TIMED,
        ACTIVE_FLAG
    }

    private ShootState shootState = ShootState.IDLE;
    private ShootMode shootMode = ShootMode.NONE;
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

                hood.setAngle(Target_Hoodposition_Angle);
                shooter.startShooter(shooterTargetVelocity);
                
                if (shooter.isVelocityWithinThreshold() || now - shootStateStartMs >= spinupTimeoutMs) {
                    gate.openGate();
                    shootState = ShootState.SHOOTING;
                    shootStateStartMs = now;
                }
                break;

            case SHOOTING:
                gate.openGate();
                spinner.turnOn();
                
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
                    spinner.turnOff();
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
            this.spinner.turnOff();
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
            this.spinner.turnOff();
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
                spinner.turnOff();
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
        spinner.turnOn();
    }

    public void stopIntake() {
        intakeActive = false;
        if(shootState != ShootState.SHOOTING) {
            spinner.turnOff();
        }        
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
