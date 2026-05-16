package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;

/**
 * CameraController wraps the Limelight 3A and provides robot pose estimates in
 * Pedro Pathing field coordinates (inches, radians).
 *
 * Uses IMU orientation to tell Limelight the robot's heading, then transforms
 * Limelight's reported pose from Limelight frame (meters, origin at field center)
 * to field frame (inches, origin at lower-left corner).
 *
 * ── COORDINATE TRANSFORMATION ─────────────────────────────────────────────────
 * Limelight coordinates: origin at field center (0,0), meters.
 * Field coordinates: origin at lower-left (0,0), inches.
 * Field center: (72, 72) inches.
 *
 * Mapping:
 *   fieldX = FIELD_CENTER_IN + METRE_INCH_MULTIPLIER * limelightY
 *   fieldY = FIELD_CENTER_IN - METRE_INCH_MULTIPLIER * limelightX
 *   fieldHeading = transform applied based on limelightHeading ranges
 *
 * ── PIPELINES ─────────────────────────────────────────────────────────────────
 *   Pipeline 0 (RED_GOAL_PIPELINE)  → detects red alliance goal-post tag.
 *   Pipeline 1 (BLUE_GOAL_PIPELINE) → detects blue alliance goal-post tag.
 *
 * ── HARDWARE MAP NAME ─────────────────────────────────────────────────────────
 *   Default device name is "limelight". Configure in the Driver Hub robot config.
 */
public class CameraController {

    // ── Pipeline indices ───────────────────────────────────────────────────────
    /** Pipeline 0: custom pipeline that detects ONLY the red alliance goal-post tag. */
    public static final int RED_GOAL_PIPELINE  = 0;
    /** Pipeline 1: custom pipeline that detects ONLY the blue alliance goal-post tag. */
    public static final int BLUE_GOAL_PIPELINE = 1;

    // ── DECODE field tag IDs (FTC DECODE TU32) ────────────────────────────────
    public static final int DECODE_BLUE_GOAL_TAG_ID = 20;
    public static final int DECODE_RED_GOAL_TAG_ID  = 24;

    // ── Field and coordinate transformation constants ──────────────────────────
    /** Conversion factor from meters to inches. */
    public static final double METRE_INCH_MULTIPLIER = 39.3701;
    /**
     * Field center in inches, derived from ParamsConfig so it stays in sync with
     * {@link ParamsConfig#FIELD_SIZE_IN} if the field size ever changes.
     */
    public static final double FIELD_CENTER_IN = ParamsConfig.FIELD_SIZE_IN / 2.0;

    // ── Internal state ────────────────────────────────────────────────────────
    private Limelight3A limelight;
    private IMU imu;
    private boolean isInitialized = false;
    private int activePipeline = RED_GOAL_PIPELINE;

    // Per-pipeline expected tag IDs (for result validation)
    private final Set<Integer> pipeline0Tags = new HashSet<>();   // red goal
    private final Set<Integer> pipeline1Tags = new HashSet<>();   // blue goal

    // Cached pose data
    private Pose3D last_botpose = null;
    private LLResult pose_result = null;
    private long last_updatedtime = 0;
    private Pose lastValidPose = null;
    /** Latest raw result from getLatestResult(), valid or not. Used for tag queries. */
    private LLResult lastRawResult = null;

    public CameraController() {
        pipeline0Tags.add(DECODE_RED_GOAL_TAG_ID);
        pipeline1Tags.add(DECODE_BLUE_GOAL_TAG_ID);
    }

    // ── Initialisation ────────────────────────────────────────────────────────

    /**
     * Initialise with a hardware map, IMU, and starting pipeline.
     *
     * @param hardwareMap OpMode hardware map
     * @param imu         Robot IMU for orientation updates
     * @param pipeline    Starting pipeline index (RED_GOAL_PIPELINE or BLUE_GOAL_PIPELINE)
     */
    public void init(HardwareMap hardwareMap, IMU imu, int pipeline) {
        this.imu = imu;
        limelight = hardwareMap.get(Limelight3A.class, ControllerParams.HW_LIMELIGHT);
        limelight.setPollRateHz(ControllerParams.CAMERA_POLL_RATE_HZ);
        limelight.pipelineSwitch(pipeline);
        activePipeline = pipeline;
        limelight.start();
        isInitialized = true;
    }

    // ── Pose configuration ────────────────────────────────────────────────────

    /**
     * Update the Limelight with current robot orientation from IMU.
     * Call this each loop cycle before reading pose estimates.
     *
     * @param current_ms Current time in milliseconds
     * @return The latest LLResult, or null if no valid pose is available
     */
    public LLResult update(long current_ms) {
        if (!isInitialized || imu == null) return null;

        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        limelight.updateRobotOrientation(orientation.getYaw(AngleUnit.DEGREES));

        LLResult result = limelight.getLatestResult();
        lastRawResult = result;
        if (result != null && result.isValid()) {
            pose_result = result;
            last_botpose = result.getBotpose();
            last_updatedtime = current_ms;
            return pose_result;
        }

        return null;
    }

    // ── Pipeline management ───────────────────────────────────────────────────

    /** Switch to pipeline 0 — red alliance goal-post tag. */
    public void switchToRedGoalPipeline() {
        setPipeline(RED_GOAL_PIPELINE);
    }

    /** Switch to pipeline 1 — blue alliance goal-post tag. */
    public void switchToBlueGoalPipeline() {
        setPipeline(BLUE_GOAL_PIPELINE);
    }

    /** Switch to an arbitrary pipeline index and update internal tracking. */
    public void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
            activePipeline = pipelineIndex;
        }
    }

    /** Returns the currently active pipeline index. */
    public int getActivePipeline() {
        return activePipeline;
    }

    // ── Pose estimation ───────────────────────────────────────────────────────

    /**
     * Returns the latest robot pose in Pedro Pathing field coordinates (inches, radians).
     *
     * Transforms Limelight's pose from Limelight frame (meters, origin at field center)
     * to field frame (inches, origin lower-left).
     *
     * Returns null when no valid pose is available.
     */
    public Pose getRobotPose() {
        if (!isInitialized || last_botpose == null) return null;
        if (System.currentTimeMillis() - last_updatedtime > ControllerParams.CAMERA_POSE_STALENESS_MS) {
            return null;
        }

        // Convert Limelight meters to inches
        double limelightX_in = METRE_INCH_MULTIPLIER * last_botpose.getPosition().y;
        double limelightY_in = -METRE_INCH_MULTIPLIER * last_botpose.getPosition().x;

        // Transform to field frame (origin lower-left)
        double fieldX = FIELD_CENTER_IN + limelightX_in;
        double fieldY = FIELD_CENTER_IN + limelightY_in;

        // Rotate from Limelight frame to field frame by subtracting the fixed offset.
        // BUG FIX: the previous if/else chain correctly applied (yaw - 90) for positive
        // angles but incorrectly applied (90 + |yaw|) = (90 - yaw) for negative angles,
        // flipping the sign. Simplified to a single subtraction that is correct for all
        // yaw values in [-180, 180].
        double fieldYawDeg = last_botpose.getOrientation().getYaw(AngleUnit.DEGREES)
                - ControllerParams.CAMERA_HEADING_OFFSET_DEG;

        lastValidPose = new Pose(fieldX, fieldY, Math.toRadians(fieldYawDeg));
        return lastValidPose;
    }

    /**
     * Returns the camera pose reported by the Limelight, converted to inches.
     * For compatibility—use getRobotPose() for the actual field-frame pose.
     *
     * Returns null when no valid pose is available.
     */
    public Pose getCameraRawPose() {
        if (!isInitialized) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        double xInches = METRE_INCH_MULTIPLIER * botpose.getPosition().x;
        double yInches = METRE_INCH_MULTIPLIER * botpose.getPosition().y;
        double cameraHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(xInches, yInches, cameraHeading);
    }

    /**
     * Returns the last successfully computed robot pose.
     * May be stale — check {@link #getTagCount()} before trusting for navigation.
     */
    public Pose getLastValidPose() {
        return lastValidPose;
    }

    /** Returns true if a fresh valid robot pose is available this cycle. */
    public boolean hasPoseEstimate() {
        return getRobotPose() != null;
    }

    /**
     * Returns the cached raw Limelight pose (Pose3D in Limelight frame, meters).
     */
    public Pose3D getLast_botpose() {
        return last_botpose;
    }

    /**
     * Returns the cached Limelight result object.
     */
    public LLResult getLast_botposeResult() {
        return pose_result;
    }

    /**
     * Returns the timestamp of the last successful pose update.
     */
    public long getLast_updatedTime() {
        return last_updatedtime;
    }

    // ── Tag inspection ────────────────────────────────────────────────────────

    /** Returns the number of AprilTags detected in the latest frame. */
    public int getTagCount() {
        if (!isInitialized || lastRawResult == null || !lastRawResult.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fiducials = lastRawResult.getFiducialResults();
        return (fiducials == null) ? 0 : fiducials.size();
    }

    /**
     * Returns the raw fiducial results from the latest frame.
     * Returns null if no result is available.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (!isInitialized || lastRawResult == null || !lastRawResult.isValid()) return null;
        return lastRawResult.getFiducialResults();
    }

    /** Returns true if the tag with the given ID is currently visible. */
    public boolean isTagVisible(int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null) return false;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == tagId) return true;
        }
        return false;
    }

    /** Returns true if the red alliance goal-post tag is currently visible. */
    public boolean isRedGoalTagVisible() {
        return isTagVisible(DECODE_RED_GOAL_TAG_ID);
    }

    /** Returns true if the blue alliance goal-post tag is currently visible. */
    public boolean isBlueGoalTagVisible() {
        return isTagVisible(DECODE_BLUE_GOAL_TAG_ID);
    }

    // ── Lifecycle ─────────────────────────────────────────────────────────────

    /** Stop the Limelight (call in OpMode.stop() to reduce CPU and power draw). */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public boolean isInitialized() {
        return isInitialized;
    }

    // ── Private helpers ───────────────────────────────────────────────────────

    /** Returns the expected tag ID set for the active pipeline. */
    private Set<Integer> getActivePipelineTags() {
        return (activePipeline == BLUE_GOAL_PIPELINE) ? pipeline1Tags : pipeline0Tags;
    }

    private boolean containsAnyTag(List<LLResultTypes.FiducialResult> fiducials, Set<Integer> ids) {
        if (ids.isEmpty()) return true;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (ids.contains(f.getFiducialId())) return true;
        }
        return false;
    }
}
