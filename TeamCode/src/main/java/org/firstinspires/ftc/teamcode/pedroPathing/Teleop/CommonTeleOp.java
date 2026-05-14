package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.CameraController;
import java.util.Locale;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.ShootSequencer;

public abstract class CommonTeleOp extends OpMode {
    public enum DriveMode {
        SEMI_AUTON,
        MANUAL_ASSISTED,
        FULLY_MANUAL
    }

    protected Follower follower;
    protected CameraController cameraController;
    protected ShootSequencer shootSequencer;
    protected TelemetryManager telemetryM;

    private final ShootingLookupTable lookupTable = new ShootingLookupTable();

    private DriveMode driveMode = DriveMode.SEMI_AUTON;
    private boolean modeSelectionLocked = true;
    private boolean manualAdjustmentsLocked = true;

    private int selectedPresetIndex = 0;

    private boolean autoDriveActive = false;
    private boolean assistedAlignActive = false;

    private boolean prevGp1DpadUp;
    private boolean prevGp1DpadDown;
    private boolean prevGp1DpadLeft;
    private boolean prevGp1DpadRight;
    private boolean prevGp1A;
    private boolean prevGp1B;
    private boolean prevGp1X;
    private boolean prevGp1Y;
    private boolean prevGp1LeftBumper;

    private boolean prevGp2DpadUp;
    private boolean prevGp2DpadDown;
    private boolean prevGp2DpadLeft;
    private boolean prevGp2DpadRight;
    private boolean prevGp2Y;
    private boolean prevGp2A;
    private boolean prevGp2X;
    private boolean prevGp2B;
    private boolean prevGp2LeftBumper;
    private boolean prevGp2RightBumper;
    private boolean prevGp2Back;

    private boolean lastShootHeld;

    private double targetManualCloseRpm = TeleOpTuningConfig.TARGET_MANUAL_CLOSE_RPM;
    private double targetManualCloseHoodDeg = TeleOpTuningConfig.TARGET_MANUAL_CLOSE_HOOD_DEG;
    private double targetManualFarRpm = TeleOpTuningConfig.TARGET_MANUAL_FAR_RPM;
    private double targetManualFarHoodDeg = TeleOpTuningConfig.TARGET_MANUAL_FAR_HOOD_DEG;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        cameraController = new CameraController();
        cameraController.init(hardwareMap, getAllianceConfig().cameraPipeline);
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

        updateModeControls();
        updatePresetSelectionAndAutoDrive();
        updateIntakeControls();

        if (assistedAlignActive) {
            runAssistedHeadingAlign();
        }

        if (!autoDriveActive && !assistedAlignActive) {
            follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * TeleOpTuningConfig.STICK_SCALE,
                    -gamepad1.left_stick_x * TeleOpTuningConfig.STICK_SCALE,
                    -gamepad1.right_stick_x * TeleOpTuningConfig.STICK_SCALE,
                    true
            );
        }

        if (autoDriveActive && (gamepad1.left_bumper && !prevGp1LeftBumper || !follower.isBusy())) {
            cancelAutoDrive();
        }

        updateShootingControls();
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

    private void updateModeControls() {
        boolean gp1DpadLeftEdge = gamepad1.dpad_left && !prevGp1DpadLeft;
        boolean gp1DpadRightEdge = gamepad1.dpad_right && !prevGp1DpadRight;
        boolean gp1DpadUpEdge = gamepad1.dpad_up && !prevGp1DpadUp;
        boolean gp1DpadDownEdge = gamepad1.dpad_down && !prevGp1DpadDown;

        if (gp1DpadLeftEdge) {
            modeSelectionLocked = true;
        }
        if (gp1DpadRightEdge) {
            modeSelectionLocked = false;
        }

        if (!modeSelectionLocked) {
            if (gp1DpadUpEdge) {
                cycleMode(1);
            } else if (gp1DpadDownEdge) {
                cycleMode(-1);
            }
        }

        boolean gp2LeftBumperEdge = gamepad2.left_bumper && !prevGp2LeftBumper;
        boolean gp2RightBumperEdge = gamepad2.right_bumper && !prevGp2RightBumper;
        if (gp2LeftBumperEdge) {
            manualAdjustmentsLocked = true;
        }
        if (gp2RightBumperEdge) {
            manualAdjustmentsLocked = false;
        }
    }

    private void cycleMode(int delta) {
        DriveMode[] values = DriveMode.values();
        int next = (driveMode.ordinal() + delta) % values.length;
        if (next < 0) {
            next += values.length;
        }
        driveMode = values[next];
    }

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
        switch (driveMode) {
            case SEMI_AUTON:
                runSemiAutonShootControls();
                break;
            case MANUAL_ASSISTED:
                runManualAssistedShootControls();
                break;
            case FULLY_MANUAL:
                runFullyManualShootControls();
                break;
        }
    }

    private void runSemiAutonShootControls() {
        TeleOpTuningConfig.ShotConfig presetShot = getPresetShotConfig();
        boolean shootHeld = gamepad2.right_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD
            || gamepad2.left_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;
        runHoldToShoot(shootHeld, presetShot.rpm, presetShot.hoodPosition);
    }

    private void runManualAssistedShootControls() {
        boolean assistedButtonEdge = gamepad2.back && !prevGp2Back;
        if (assistedButtonEdge) {
            assistedAlignActive = true;
            autoDriveActive = false;
            shootSequencer.stopShootingSequence();
        }

        if (!assistedAlignActive) {
            lastShootHeld = false;
        }
    }

    private void runFullyManualShootControls() {
        if (!manualAdjustmentsLocked) {
            boolean gp2DpadUpEdge = gamepad2.dpad_up && !prevGp2DpadUp;
            boolean gp2DpadDownEdge = gamepad2.dpad_down && !prevGp2DpadDown;
            boolean gp2DpadLeftEdge = gamepad2.dpad_left && !prevGp2DpadLeft;
            boolean gp2DpadRightEdge = gamepad2.dpad_right && !prevGp2DpadRight;
            boolean gp2YEdge = gamepad2.y && !prevGp2Y;
            boolean gp2AEdge = gamepad2.a && !prevGp2A;
            boolean gp2XEdge = gamepad2.x && !prevGp2X;
            boolean gp2BEdge = gamepad2.b && !prevGp2B;

            if (gp2DpadUpEdge) {
                targetManualCloseRpm += TeleOpTuningConfig.CLOSE_RPM_STEP;
            }
            if (gp2DpadDownEdge) {
                targetManualCloseRpm = Math.max(0.0, targetManualCloseRpm - TeleOpTuningConfig.CLOSE_RPM_STEP);
            }
            if (gp2DpadRightEdge) {
                targetManualCloseHoodDeg += TeleOpTuningConfig.CLOSE_HOOD_STEP_DEG;
            }
            if (gp2DpadLeftEdge) {
                targetManualCloseHoodDeg = Math.max(0.0, targetManualCloseHoodDeg - TeleOpTuningConfig.CLOSE_HOOD_STEP_DEG);
            }

            if (gp2YEdge) {
                targetManualFarRpm += TeleOpTuningConfig.FAR_RPM_STEP;
            }
            if (gp2AEdge) {
                targetManualFarRpm = Math.max(0.0, targetManualFarRpm - TeleOpTuningConfig.FAR_RPM_STEP);
            }
            if (gp2XEdge) {
                targetManualFarHoodDeg += TeleOpTuningConfig.FAR_HOOD_STEP_DEG;
            }
            if (gp2BEdge) {
                targetManualFarHoodDeg = Math.max(0.0, targetManualFarHoodDeg - TeleOpTuningConfig.FAR_HOOD_STEP_DEG);
            }
        }

        boolean farShootHeld = gamepad2.right_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;
        boolean closeShootHeld = gamepad2.left_trigger > TeleOpTuningConfig.TRIGGER_THRESHOLD;

        if (farShootHeld) {
            runHoldToShoot(true, targetManualFarRpm, targetManualFarHoodDeg);
            return;
        }

        if (closeShootHeld) {
            runHoldToShoot(true, targetManualCloseRpm, targetManualCloseHoodDeg);
            return;
        }

        runHoldToShoot(false, 0.0, 0.0);
    }

    private void runHoldToShoot(boolean shootHeld, double targetRpm, double targetHoodAngleDeg) {
        shootSequencer.setHoodPosition(targetHoodAngleDeg); // Now targetHoodAngleDeg is actually position 0.0-1.0
        if (shootHeld && !lastShootHeld) {
            shootSequencer.startShootingSequence(targetRpm, TeleOpTuningConfig.SHOOTER_VELOCITY_THRESHOLD_RPM);
        }
        if (!shootHeld && lastShootHeld) {
            shootSequencer.stopShootingSequence();
        }
        lastShootHeld = shootHeld;
    }

    private void runAssistedHeadingAlign() {
        Pose aimPose = cameraController.getRobotPose();
        if (aimPose == null) {
            aimPose = follower.getPose();
        }

        Pose goalPose = getAllianceConfig().goalPose;
        double targetHeading = Math.atan2(goalPose.getY() - aimPose.getY(), goalPose.getX() - aimPose.getX());
        double headingError = normalizeRadians(targetHeading - follower.getPose().getHeading());

        if (Math.abs(headingError) <= TeleOpTuningConfig.HEADING_ALIGN_TOLERANCE_RAD) {
            assistedAlignActive = false;
                ShootingLookupTable.ShotSolution shot = lookupTable.getNearest(distanceToGoalInches(aimPose));
                shootSequencer.setHoodPosition(shot.hoodPosition);
                shootSequencer.startShootingSequence(
                    TeleOpTuningConfig.ASSISTED_SHOOT_DURATION_MS,
                    shot.rpm,
                    TeleOpTuningConfig.SHOOTER_VELOCITY_THRESHOLD_RPM
                );
            lastShootHeld = false;
            return;
        }

        double turn = clamp(
                headingError * TeleOpTuningConfig.HEADING_ALIGN_KP,
                -TeleOpTuningConfig.HEADING_ALIGN_MAX_TURN,
                TeleOpTuningConfig.HEADING_ALIGN_MAX_TURN
        );
        follower.setTeleOpDrive(0.0, 0.0, turn, true);
    }

    private TeleOpTuningConfig.ShotConfig getPresetShotConfig() {
        TeleOpTuningConfig.ShotConfig[] configs = getAllianceConfig().presetShots;
        if (selectedPresetIndex < 0 || selectedPresetIndex >= configs.length) {
            return configs[0];
        }
        return configs[selectedPresetIndex];
    }

    private double distanceToGoalInches(Pose robotPose) {
        Pose goal = getAllianceConfig().goalPose;
        double dx = goal.getX() - robotPose.getX();
        double dy = goal.getY() - robotPose.getY();
        return Math.hypot(dx, dy);
    }

    private void updateTelemetry() {
        telemetry.addData("mode", driveMode);
        telemetry.addData("modeLocked", modeSelectionLocked);
        telemetry.addData("manualAdjustLocked", manualAdjustmentsLocked);
        telemetry.addData("selectedPreset", selectedPresetIndex);
        telemetry.addData("autoDriveActive", autoDriveActive);
        telemetry.addData("assistedAlignActive", assistedAlignActive);

        telemetry.addData("manualCloseRPM", targetManualCloseRpm);
        telemetry.addData("manualCloseHood", targetManualCloseHoodDeg);
        telemetry.addData("manualFarRPM", targetManualFarRpm);
        telemetry.addData("manualFarHood", targetManualFarHoodDeg);

        telemetry.addData("pose", String.format(Locale.US, "(%.2f, %.2f, %.2f)",
            follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading())));
        Pose campose = cameraController.getRobotPose();
        if (campose != null) telemetry.addData("Campose", String.format(Locale.US, "(%.2f, %.2f, %.2f)",
            campose.getX(), campose.getY(), Math.toDegrees(campose.getHeading())));
        telemetry.addData("cameraTagCount", cameraController.getTagCount());


        telemetryM.debug("mode", driveMode);
        telemetryM.debug("modeLocked", modeSelectionLocked);
        telemetryM.debug("manualAdjustLocked", manualAdjustmentsLocked);
        telemetryM.debug("selectedPreset", selectedPresetIndex);
        telemetryM.debug("autoDriveActive", autoDriveActive);
        telemetryM.debug("assistedAlignActive", assistedAlignActive);
        telemetryM.debug("pose", String.format(Locale.US, "(%.2f, %.2f, %.2f)",
            follower.getPose().getX(), follower.getPose().getY(), Math.toDegrees(follower.getPose().getHeading())));
        Pose camposeDbg = cameraController.getRobotPose();
        if (camposeDbg != null) telemetryM.debug("Campose", String.format(Locale.US, "(%.2f, %.2f, %.2f)",
            camposeDbg.getX(), camposeDbg.getY(), Math.toDegrees(camposeDbg.getHeading())));
    }

    private void cacheButtonStates() {
        prevGp1DpadUp = gamepad1.dpad_up;
        prevGp1DpadDown = gamepad1.dpad_down;
        prevGp1DpadLeft = gamepad1.dpad_left;
        prevGp1DpadRight = gamepad1.dpad_right;
        prevGp1A = gamepad1.a;
        prevGp1B = gamepad1.b;
        prevGp1X = gamepad1.x;
        prevGp1Y = gamepad1.y;
        prevGp1LeftBumper = gamepad1.left_bumper;

        prevGp2DpadUp = gamepad2.dpad_up;
        prevGp2DpadDown = gamepad2.dpad_down;
        prevGp2DpadLeft = gamepad2.dpad_left;
        prevGp2DpadRight = gamepad2.dpad_right;
        prevGp2Y = gamepad2.y;
        prevGp2A = gamepad2.a;
        prevGp2X = gamepad2.x;
        prevGp2B = gamepad2.b;
        prevGp2LeftBumper = gamepad2.left_bumper;
        prevGp2RightBumper = gamepad2.right_bumper;
        prevGp2Back = gamepad2.back;
    }

    private double normalizeRadians(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
