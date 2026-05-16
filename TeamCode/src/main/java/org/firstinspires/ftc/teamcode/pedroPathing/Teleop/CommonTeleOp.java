package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.CameraController;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.ShootSequencer;

public abstract class CommonTeleOp extends OpMode {
    protected Follower follower;
    protected CameraController cameraController;
    protected ShootSequencer shootSequencer;
    protected TelemetryManager telemetryM;

    private final ShootingLookupTable lookupTable = new ShootingLookupTable();

    private int selectedPresetIndex = 0;

    private boolean autoDriveActive = false;
    private boolean assistedAlignActive = false;

    private boolean prevGp1A;
    private boolean prevGp1B;
    private boolean prevGp1X;
    private boolean prevGp1Y;
    private boolean prevGp1LeftBumper;
    private boolean prevGp2LeftTriggerActive;

    private boolean lastShootHeld;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        cameraController = new CameraController();
        IMU imu = hardwareMap.get(IMU.class, "imu");
        cameraController.init(hardwareMap, imu, getAllianceConfig().cameraPipeline);
        shootSequencer = new ShootSequencer();
        shootSequencer.init(hardwareMap);
        shootSequencer.setShooterRunningAfterShoot(true);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        follower.setStartingPose(getAllianceConfig().startPose);
        follower.update();
    }

    @Override
    public void start() {
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        follower.update();
        cameraController.update(System.currentTimeMillis());

        updatePresetSelectionAndAutoDrive();
        updateIntakeControls();

        if (autoDriveActive && (gamepad1.left_bumper && !prevGp1LeftBumper || !follower.isBusy())) {
            cancelAutoDrive();
        }

        updateShootingControls();

        if (assistedAlignActive && !follower.isBusy()) {
            cancelAssistedAlign();
        }

        if (!autoDriveActive && !assistedAlignActive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * TeleOpTuningConfig.STICK_SCALE,
                    -gamepad1.left_stick_x * TeleOpTuningConfig.STICK_SCALE,
                    -gamepad1.right_stick_x * TeleOpTuningConfig.STICK_SCALE,
                    true
            );
        }

        shootSequencer.loop(lastShootHeld);
        updateTelemetry();

        cacheButtonStates();
        telemetryM.update();
        telemetry.update();
    }

    @Override
    public void stop() {
        shootSequencer.stopIntake();
        shootSequencer.setShooterRunningAfterShoot(false);
        shootSequencer.stopShootingSequence();
        cameraController.stop();
    }

    protected abstract TeleOpTuningConfig.AllianceTeleOpConfig getAllianceConfig();

    private void updatePresetSelectionAndAutoDrive() {
        boolean gp1AEdge = gamepad1.a && !prevGp1A;
        boolean gp1BEdge = gamepad1.b && !prevGp1B;
        boolean gp1XEdge = gamepad1.x && !prevGp1X;
        boolean gp1YEdge = gamepad1.y && !prevGp1Y;

        if (gp1AEdge) {
            driveToPreset(0);
        } else if (gp1BEdge) {
            driveToPreset(1);
        } else if (gp1XEdge) {
            driveToPreset(2);
        } else if (gp1YEdge) {
            driveToPreset(3);
        }
    }

    private void driveToPreset(int index) {
        Pose[] poses = getAllianceConfig().presetPoses;
        if (index < 0 || index >= poses.length) {
            return;
        }

        selectedPresetIndex = index;
        Pose target = poses[index];
        PathChain goToPose = follower.pathBuilder()
                .addPath(new BezierLine(follower::getPose, target))
                .setLinearHeadingInterpolation(follower.getPose().getHeading(), target.getHeading())
                .build();

        follower.followPath(goToPose, true);
        autoDriveActive = true;
    }

    private void cancelAutoDrive() {
        follower.startTeleopDrive();
        autoDriveActive = false;
        assistedAlignActive = false;
    }

    private void cancelAssistedAlign() {
        follower.startTeleopDrive();
        assistedAlignActive = false;
    }

    private void updateIntakeControls() {
        if (gamepad1.right_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD) {
            shootSequencer.startIntake();
            return;
        }

        if (gamepad1.left_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD) {
            shootSequencer.stopIntake();
            shootSequencer.startIntakeReverse();
            return;
        }

        shootSequencer.stopIntake();
    }

    private void updateShootingControls() {
        boolean presetShootHeld = gamepad2.right_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;
        boolean aimShootHeld = gamepad2.left_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;
        boolean aimShootEdge = aimShootHeld && !prevGp2LeftTriggerActive;
        boolean longShootHeld = gamepad2.right_bumper;
        boolean closeShootHeld = gamepad2.left_bumper;

        if (presetShootHeld) {
            TeleOpTuningConfig.ShotConfig presetShot = getPresetShotConfig();
            if (assistedAlignActive) {
                cancelAssistedAlign();
            }
            runHoldToShoot(true, presetShot.rpm, presetShot.hoodPosition);
            return;
        }

        if (aimShootEdge) {
            runCameraAimedShootControls();
            return;
        }

        if (aimShootHeld) {
            return;
        }

        if (longShootHeld) {
            if (assistedAlignActive) {
                cancelAssistedAlign();
            }
            runHoldToShoot(true,
                    ParamsConfig.TELEOP_SHOOT_TARGET_VELOCITY_LONG,
                    ParamsConfig.TELEOP_SHOOT_HOOD_POSITION_LONG);
            return;
        }

        if (closeShootHeld) {
            if (assistedAlignActive) {
                cancelAssistedAlign();
            }
            runHoldToShoot(true,
                    ParamsConfig.TELEOP_SHOOT_TARGET_VELOCITY_CLOSE,
                    ParamsConfig.TELEOP_SHOOT_HOOD_POSITION_CLOSE);
            return;
        }

        runHoldToShoot(false, 0.0, ParamsConfig.TELEOP_SHOOT_HOOD_POSITION_CLOSE);
    }

    private void runHoldToShoot(boolean shootHeld, double targetRpm, double targetHoodPosition) {
        if (shootHeld) {
            shootSequencer.setShooterTargetVelocity(targetRpm);
            shootSequencer.setHoodPosition(targetHoodPosition);
        }
        if (shootHeld && !shootSequencer.isShootingSequenceActive()) {
            shootSequencer.startShootingSequence(targetRpm, TeleOpTuningConfig.SHOOTER_VELOCITY_THRESHOLD_RPM);
        }
        if (!shootHeld && lastShootHeld) {
            shootSequencer.stopShootingSequence();
        }
        lastShootHeld = shootHeld;
    }

    private void PedroPathing_TurnByAngle(double targetCorrectionRad) {

            double turnAngle = normalizeRadians(targetCorrectionRad);
            boolean turnPositive = turnAngle < 0;
            follower.turn(turnAngle, turnPositive);

    }
    


    private void runCameraAimedShootControls() {
        Pose goalPose = getAllianceConfig().goalPose;
        CameraController.AimData aimData = cameraController.getAimData(goalPose);
        Pose currentPose = follower.getPose();

        if (!aimData.isValid || cameraController.getTagCount() == 0 || currentPose == null) {
            cancelAssistedAlign();
            return;
        }

        assistedAlignActive = true;
        double headingCorrectionRad = Math.toRadians(aimData.headingCorrectionDeg);

        if (Math.abs(headingCorrectionRad) <= TeleOpTuningConfig.HEADING_ALIGN_TOLERANCE_RAD) {
            ShootingLookupTable.ShotSolution shot = lookupTable.getNearest(aimData.distanceToTargetInches);
            runHoldToShoot(true, shot.rpm, shot.hoodPosition);
            return;
        }

        PedroPathing_TurnByAngle(headingCorrectionRad);
        lastShootHeld = false;
    }

    private TeleOpTuningConfig.ShotConfig getPresetShotConfig() {
        TeleOpTuningConfig.ShotConfig[] configs = getAllianceConfig().presetShots;
        if (configs.length == 0) {
            return new TeleOpTuningConfig.ShotConfig(0.0, 0.0);
        }
        if (selectedPresetIndex < 0 || selectedPresetIndex >= configs.length) {
            return configs[0];
        }
        return configs[selectedPresetIndex];
    }

    private void updateTelemetry() {
        Pose goalPose = getAllianceConfig().goalPose;
        CameraController.AimData aimData = cameraController.getAimData(goalPose);
        boolean aimShootActive = gamepad2.left_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;

        telemetry.addData("selectedPreset", selectedPresetIndex);
        telemetry.addData("autoDriveActive", autoDriveActive);
        telemetry.addData("assistedAlignActive", assistedAlignActive);

        // Shooter subsystem telemetry
        telemetry.addData("shootMode", shootSequencer.getShootMode());
        telemetry.addData("shootTimed", shootSequencer.isTimedMode());
        telemetry.addData("targetRpm", shootSequencer.getShooterTargetVelocity());
        telemetry.addData("shooterState", shootSequencer.getShootState());
        telemetry.addData("shooterRpm", shootSequencer.getShooterVelocityRpm());
        telemetry.addData("hoodPos", shootSequencer.getHoodPosition());
        telemetry.addData("gatePosRaw", shootSequencer.getGatePosition());
        telemetry.addData("gateState", shootSequencer.isGateOpen() ? "OPEN" : "CLOSED");
        telemetry.addData("spinnerState", shootSequencer.getSpinnerState());
        telemetry.addData("intakeState", shootSequencer.getIntakeState());

        telemetry.addData("pose", String.format(Locale.US, "(%.2f, %.2f, %.2f)",
            follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading())));
        Pose campose = cameraController.getRobotPose();
        if (campose != null) telemetry.addData("Campose", String.format(Locale.US, "(%.2f, %.2f, %.2f)",
            campose.getX(), campose.getY(), Math.toDegrees(campose.getHeading())));
        telemetry.addData("cameraTagCount", cameraController.getTagCount());
        telemetry.addData("aimShootActive", aimShootActive);
        telemetry.addData("aimHeadingCorrectionDeg", aimData.headingCorrectionDeg);
        telemetry.addData("aimDistanceIn", aimData.distanceToTargetInches);


    }

    private void cacheButtonStates() {
        prevGp1A = gamepad1.a;
        prevGp1B = gamepad1.b;
        prevGp1X = gamepad1.x;
        prevGp1Y = gamepad1.y;
        prevGp1LeftBumper = gamepad1.left_bumper;
        prevGp2LeftTriggerActive = gamepad2.left_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;
    }

    private double normalizeRadians(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
