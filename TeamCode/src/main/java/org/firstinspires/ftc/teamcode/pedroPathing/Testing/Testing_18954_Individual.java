package org.firstinspires.ftc.teamcode.pedroPathing.Testing;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.ShootSequencer;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.CameraController;
import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.Teleop.TeleOpTuningConfig;

import com.pedropathing.geometry.Pose;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;



@TeleOp
public class Testing_18954_Individual extends OpMode{


    // ---------------- HARDWARE DECLARATION ----------------

    ShootSequencer shootSequencer;
    CameraController cameraController;

    private double shooterTargetVelocity = ParamsConfig.SHOOTER_DEFAULT_TARGET_RPM;
    private static final double SHOOTER_VELOCITY_STEP = 50.0;
    private static final double SHOOTER_VELOCITY_THRESHOLD = 50.0;
    private static final double POSITION_STEP = 0.1;

    private boolean intakeEnabled = false;
    private boolean spinnerEnabled = false;

    private boolean hoodTargetInitialized = false;
    private boolean gateTargetInitialized = false;
    private double targetHoodPosition = 0.0;
    private double targetGatePosition = 0.0;

    private boolean prevLeftBumper = false;
    private boolean prevRightBumper = false;
    private boolean prevY = false;
    private boolean prevX = false;
    private boolean prevDpadUp = false;
    private boolean prevDpadDown = false;
    private boolean prevDpadLeft = false;
    private boolean prevDpadRight = false;
    private boolean prevRightTriggerPressed = false;
    private boolean prevLeftTriggerPressed = false;


    // ---------------- INIT METHOD ----------------
    public void init() {


                // Private init tasks can be added here if needed        
        shootSequencer = new ShootSequencer();
        shootSequencer.init(hardwareMap);
        shootSequencer.setShooterVelocityThreshold(SHOOTER_VELOCITY_THRESHOLD);

        cameraController = new CameraController();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        cameraController.init(hardwareMap, imu, CameraController.RED_GOAL_PIPELINE);


        telemetry.addData("Status", "Initialized");      
    }


    // ---------------- LOOP METHOD ----------------

    public void loop() {
        cameraController.update(System.currentTimeMillis());

        if (!hoodTargetInitialized) {
            targetHoodPosition = shootSequencer.getHoodPosition();
            hoodTargetInitialized = true;
        }

        if (!gateTargetInitialized) {
            targetGatePosition = shootSequencer.getGatePosition();
            gateTargetInitialized = true;
        }

        boolean leftBumperPressed = gamepad1.left_bumper && !prevLeftBumper;
        boolean rightBumperPressed = gamepad1.right_bumper && !prevRightBumper;
        boolean yPressed = gamepad1.y && !prevY;
        boolean xPressed = gamepad1.x && !prevX;
        boolean dpadUpPressed = gamepad1.dpad_up && !prevDpadUp;
        boolean dpadDownPressed = gamepad1.dpad_down && !prevDpadDown;
        boolean dpadLeftPressed = gamepad1.dpad_left && !prevDpadLeft;
        boolean dpadRightPressed = gamepad1.dpad_right && !prevDpadRight;

        boolean rightTriggerPressed = gamepad1.right_trigger > 0.5;
        boolean leftTriggerPressed = gamepad1.left_trigger > 0.5;
        boolean rightTriggerEdge = rightTriggerPressed && !prevRightTriggerPressed;
        boolean leftTriggerEdge = leftTriggerPressed && !prevLeftTriggerPressed;

        if (rightBumperPressed) {
            shooterTargetVelocity += SHOOTER_VELOCITY_STEP;
        }

        if (leftBumperPressed) {
            shooterTargetVelocity = Math.max(0.0, shooterTargetVelocity - SHOOTER_VELOCITY_STEP);
        }

        if (rightTriggerEdge) {
            shootSequencer.startShooterDirectly(shooterTargetVelocity);
        }

        if (leftTriggerEdge) {
            shootSequencer.stopShooterDirectly();
        }

        if (yPressed) {
            intakeEnabled = !intakeEnabled;
            if (intakeEnabled) {
                shootSequencer.startIntake();
            } else {
                shootSequencer.stopIntake();
            }
        }

        if (xPressed) {
            spinnerEnabled = !spinnerEnabled;
            if (spinnerEnabled) {
                shootSequencer.turnOnSpinner();
            } else {
                shootSequencer.turnOffSpinner();
            }
        }

        if (dpadUpPressed) {
            targetHoodPosition = Math.min(1.0, targetHoodPosition + POSITION_STEP);
            shootSequencer.setHoodPosition(targetHoodPosition);
        }

        if (dpadDownPressed) {
            targetHoodPosition = Math.max(0.0, targetHoodPosition - POSITION_STEP);
            shootSequencer.setHoodPosition(targetHoodPosition);
        }

        if (dpadLeftPressed) {
            targetGatePosition = Math.max(0.0, targetGatePosition - POSITION_STEP);
            shootSequencer.setGatePosition(targetGatePosition);
        }

        if (dpadRightPressed) {
            targetGatePosition = Math.min(1.0, targetGatePosition + POSITION_STEP);
            shootSequencer.setGatePosition(targetGatePosition);
        }

        double currentShooterVelocity = shootSequencer.getShooterVelocityRpm();
        boolean thresholdReached = shootSequencer.isShooterVelocityWithinThreshold();
        double currentHoodPosition = shootSequencer.getHoodPosition();
        double currentGatePosition = shootSequencer.getGatePosition();
        Pose cameraPose = cameraController.getRobotPose();
        CameraController.AimData aimData = cameraPose == null
            ? CameraController.AimData.invalid()
            : cameraController.getAimData(TeleOpTuningConfig.RED.goalPose);
        double headingCorrectionDeg = aimData.headingCorrectionDeg;

        telemetry.addData("Target Shooter Velocity (RPM)", shooterTargetVelocity);
        telemetry.addData("Current Shooter Velocity (RPM)", currentShooterVelocity);
        telemetry.addData("Shooter Threshold Reached", thresholdReached);
        telemetry.addData("Current Hood Position", currentHoodPosition);
        telemetry.addData("Current Gate Position", currentGatePosition);
        telemetry.addData("Cam Tag Count", cameraController.getTagCount());
        telemetry.addData("Cam Pose Available", cameraPose != null);
        if (cameraPose != null) {
            telemetry.addData("Bot X", cameraPose.getX());
            telemetry.addData("Bot Y", cameraPose.getY());
            telemetry.addData("Bot Heading (deg)", Math.toDegrees(cameraPose.getHeading()));
        }
        telemetry.addData("Heading Correction Needed (deg)", headingCorrectionDeg);
        telemetry.addData("Distance to Target (in)", aimData.distanceToTargetInches);
        telemetry.update();

        prevLeftBumper = gamepad1.left_bumper;
        prevRightBumper = gamepad1.right_bumper;
        prevY = gamepad1.y;
        prevX = gamepad1.x;
        prevDpadUp = gamepad1.dpad_up;
        prevDpadDown = gamepad1.dpad_down;
        prevDpadLeft = gamepad1.dpad_left;
        prevDpadRight = gamepad1.dpad_right;
        prevRightTriggerPressed = rightTriggerPressed;
        prevLeftTriggerPressed = leftTriggerPressed;

    }
}
