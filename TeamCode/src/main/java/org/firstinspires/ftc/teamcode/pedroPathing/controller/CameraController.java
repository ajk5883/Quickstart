package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.List;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;

/** Wraps the Limelight 3A pose and target-offset readings. */
public class CameraController {

    /** Pipeline 0 for the red alliance goal-post tag. */
    public static final int RED_GOAL_PIPELINE  = 0;
    /** Pipeline 1 for the blue alliance goal-post tag. */
    public static final int BLUE_GOAL_PIPELINE = 1;

    /** Conversion factor from meters to inches. */
    public static final double METRE_INCH_MULTIPLIER = 39.3701;
    /** Field center in inches. */
    public static final double FIELD_CENTER_IN = ParamsConfig.FIELD_SIZE_IN / 2.0;

    private Limelight3A limelight;
    private IMU imu;
    private boolean isInitialized = false;

    private Pose3D lastBotPose = null;
    private LLResult lastRawResult = null;
    private long lastUpdatedTime = 0;
    private Pose lastValidPose = null;

    /** Initialises the Limelight and IMU bridge. */
    public void init(HardwareMap hardwareMap, IMU imu, int pipeline) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, ControllerParams.HW_LIMELIGHT);
        limelight.setPollRateHz(ControllerParams.CAMERA_POLL_RATE_HZ);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        isInitialized = true;
    }

    /** Updates the Limelight orientation and caches the latest result. */
    public LLResult update(long current_ms) {
        if (!isInitialized || imu == null) return null;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        lastRawResult = result;
        if (result != null && result.isValid()) {
            lastBotPose = result.getBotpose();
            lastUpdatedTime = current_ms;
            return result;
        }

        return null;
    }

    /** Returns the latest robot pose in field coordinates. */
    public Pose getRobotPose() {
        if (!isInitialized || lastBotPose == null) return null;
        if (System.currentTimeMillis() - lastUpdatedTime > ControllerParams.CAMERA_POSE_STALENESS_MS) {
            return null;
        }

        double fieldX = FIELD_CENTER_IN + METRE_INCH_MULTIPLIER * lastBotPose.getPosition().y;
        double fieldY = FIELD_CENTER_IN - METRE_INCH_MULTIPLIER * lastBotPose.getPosition().x;
        double fieldYawDeg = lastBotPose.getOrientation().getYaw(AngleUnit.DEGREES)
                - ControllerParams.CAMERA_HEADING_OFFSET_DEG;

        lastValidPose = new Pose(fieldX, fieldY, Math.toRadians(fieldYawDeg));
        return lastValidPose;
    }

    /** Returns the number of fiducials in the latest valid frame. */
    public int getTagCount() {
        if (!isInitialized || lastRawResult == null || !lastRawResult.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fiducials = lastRawResult.getFiducialResults();
        return (fiducials == null) ? 0 : fiducials.size();
    }

    /** Returns the camera heading correction and target distance for shooting. */
    public AimData getAimData(Pose targetPose) {
        if (targetPose == null) {
            return AimData.invalid();
        }

        Pose robotPose = getRobotPose();
        if (robotPose == null || lastRawResult == null || !lastRawResult.isValid()) {
            return AimData.invalid();
        }

        double headingCorrectionDeg = -lastRawResult.getTx();
        double distanceToTargetInches = Math.hypot(
                targetPose.getX() - robotPose.getX(),
                targetPose.getY() - robotPose.getY());

        return new AimData(headingCorrectionDeg, distanceToTargetInches, true);
    }

    /** Stops the Limelight. */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    /** Container for camera aim correction and shooting distance. */
    public static final class AimData {
        public final double headingCorrectionDeg;
        public final double distanceToTargetInches;
        public final boolean isValid;

        private AimData(double headingCorrectionDeg, double distanceToTargetInches, boolean isValid) {
            this.headingCorrectionDeg = headingCorrectionDeg;
            this.distanceToTargetInches = distanceToTargetInches;
            this.isValid = isValid;
        }

        public static AimData invalid() {
            return new AimData(Double.NaN, Double.NaN, false);
        }
    }
}
