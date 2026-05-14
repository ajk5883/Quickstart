package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.HashSet;
import java.util.List;
import java.util.Set;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

/**
 * CameraController wraps the Limelight 3A and provides robot pose estimates in
 * Pedro Pathing field coordinates (inches, radians).
 *
 * ── ROBOT LAYOUT ──────────────────────────────────────────────────────────────
 *   • Intake  faces FORWARD  (Pedro Pathing +X direction).
 *   • Shooter + Limelight face BACKWARD (180° / π rad from intake).
 *
 * ── CAMERA MOUNT (measured from robot centre) ─────────────────────────────────
 *   • Height         : CAMERA_HEIGHT_INCHES  = 10.0 in  (above ground)
 *   • Forward offset : CAMERA_FORWARD_OFFSET = −3.543 in (90 mm BEHIND centre)
 *   • Left offset    : CAMERA_LEFT_OFFSET    = −3.543 in (90 mm to the RIGHT of
 *                      centre, when viewed from the back of the robot)
 *   • Heading offset : CAMERA_HEADING_OFFSET = π rad    (camera faces backward)
 *
 * ── PIPELINES ─────────────────────────────────────────────────────────────────
 *   Pipeline 0 (RED_GOAL_PIPELINE)  → custom pipeline, detects red goal-post tag only.
 *   Pipeline 1 (BLUE_GOAL_PIPELINE) → custom pipeline, detects blue goal-post tag only.
 *
 * ── POSE OUTPUT ──────────────────────────────────────────────────────────────
 *   The Limelight's getBotpose() is treated as the robot position reported by
 *   the camera. CameraController converts units to inches and only flips the
 *   heading by 180° in getRobotPose() so it points opposite the camera.
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

    // ── Camera mount geometry (robot-frame, all in inches / radians) ──────────
    /** Camera height above the ground (informational, not used in 2-D pose). */
    public static final double CAMERA_HEIGHT_INCHES  = 10.0;

    /**
     * Camera forward offset from robot centre (inches).
     * Negative because the camera is BEHIND the centre (shooter side).
     * 90 mm ÷ 25.4 mm/in = 3.543 in.
     */
    public static final double CAMERA_FORWARD_OFFSET = -3.543;

    /**
     * Camera left offset from robot centre (inches).
     * Negative because the camera is to the RIGHT of centre.
     * "90 mm to the right when looking from the back" → −Y in robot frame.
     */
    public static final double CAMERA_LEFT_OFFSET    = -3.543;

    /**
     * Camera heading offset relative to robot heading (radians).
     * Camera faces backward (opposite to intake) → π rad.
     */
    public static final double CAMERA_HEADING_OFFSET = Math.PI;

    // ── Internal state ────────────────────────────────────────────────────────
    private Limelight3A limelight;
    private boolean isInitialized = false;
    private int activePipeline = RED_GOAL_PIPELINE;

    // Per-pipeline expected tag IDs (for result validation)
    private final Set<Integer> pipeline0Tags = new HashSet<>();   // red goal
    private final Set<Integer> pipeline1Tags = new HashSet<>();   // blue goal

    private Pose lastValidPose = null;

    public CameraController() {
        pipeline0Tags.add(DECODE_RED_GOAL_TAG_ID);
        pipeline1Tags.add(DECODE_BLUE_GOAL_TAG_ID);
    }

    // ── Initialisation ────────────────────────────────────────────────────────


    /**
     * Initialise with a custom hardware map name and starting pipeline.
     *
     * @param hardwareMap OpMode hardware map
     * @param pipeline    Starting pipeline index
     */
    public void init(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(pipeline);
        activePipeline = pipeline;
        limelight.start();
        isInitialized = true;
    }

    // ── Pose configuration ────────────────────────────────────────────────────

    /**
     * Kept for compatibility, but no longer used by the pose calculation.
     */
    public void setCoordinateTransform(double xOffsetInches,
                                       double yOffsetInches,
                                       double headingOffsetDeg) {
    }

    /**
     * Kept for compatibility, but no longer used by the pose calculation.
     */
    public void setMirrorY(boolean mirror) {
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
     * Returns the camera pose reported by the Limelight, converted to inches.
     * Heading is left unchanged here so the robot pose can apply the 180° flip.
     *
     * Returns null when no valid pose is available.
     */
    public Pose getCameraRawPose() {
        if (!isInitialized) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        if (!containsAnyTag(fiducials, getActivePipelineTags())) return null;

        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        double xInches = botpose.getPosition().toUnit(DistanceUnit.INCH).x;
        double yInches = botpose.getPosition().toUnit(DistanceUnit.INCH).y;
        double cameraHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS);

        return new Pose(xInches, yInches, cameraHeading);
    }

    /**
     * Returns the latest robot pose in Pedro Pathing coordinates (inches, radians).
     *
     * Position is returned exactly as reported by the camera. Heading is rotated
     * by 180° so it points opposite the camera.
     *
     * Returns null when no valid pose is available.
     */
    public Pose getRobotPose() {
        Pose cameraPose = getCameraRawPose();
        if (cameraPose == null) return null;

        double robotHeading = normalizeAngle(cameraPose.getHeading() - CAMERA_HEADING_OFFSET);

        lastValidPose = new Pose(cameraPose.getX(), cameraPose.getY(), robotHeading);
        return lastValidPose;
    }

    /**
     * Returns the last successfully computed robot-centre pose.
     * May be stale — check {@link #getTagCount()} before trusting for navigation.
     */
    public Pose getLastValidPose() {
        return lastValidPose;
    }

    /** Returns true if a fresh valid robot pose is available this cycle. */
    public boolean hasPoseEstimate() {
        return getRobotPose() != null;
    }

    // ── Tag inspection ────────────────────────────────────────────────────────

    /** Returns the number of AprilTags detected in the latest frame. */
    public int getTagCount() {
        if (!isInitialized) return 0;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return (fiducials == null) ? 0 : fiducials.size();
    }

    /**
     * Returns the raw fiducial results from the latest frame.
     * Returns null if no result is available.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (!isInitialized) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result.getFiducialResults();
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

    /** Normalises an angle to [−π, π]. */
    private double normalizeAngle(double angleRad) {
        return Math.atan2(Math.sin(angleRad), Math.cos(angleRad));
    }
}
