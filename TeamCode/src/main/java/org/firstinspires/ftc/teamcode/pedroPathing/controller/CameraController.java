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
 * ── POSE TRANSFORM (two-stage) ────────────────────────────────────────────────
 *   The Limelight's getBotpose() returns the CAMERA pose in the WPI field frame
 *   (when the camera mount is not configured inside the Limelight web UI, which
 *   is the assumed default here).  CameraController applies:
 *     Stage 1 — field-to-Pedro: metres → inches, optional Y mirror, XY + heading
 *               offsets.  Result exposed via getCameraRawPose().
 *     Stage 2 — camera-to-robot-centre: rotates the physical camera mount offset
 *               into the field frame and subtracts it from the camera position.
 *               Result exposed via getRobotPose().
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

    // Stage-1 field-to-Pedro transform parameters
    private double fieldOriginOffsetX = 0.0;  // inches
    private double fieldOriginOffsetY = 0.0;  // inches
    private double headingOffsetRad   = 0.0;  // radians
    private boolean mirrorY           = false;

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

    // ── Stage-1 field-to-Pedro coordinate transform ───────────────────────────

    /**
     * Set the offset between the Limelight field origin and Pedro's (0, 0).
     *
     * Example: if the Limelight field origin is at Pedro's (−72, −72) call
     *   setCoordinateTransform(72, 72, 0).
     *
     * @param xOffsetInches    X offset (inches) added after metre → inch conversion
     * @param yOffsetInches    Y offset (inches) added after metre → inch conversion
     * @param headingOffsetDeg Heading offset (degrees) added to Limelight yaw
     */
    public void setCoordinateTransform(double xOffsetInches,
                                       double yOffsetInches,
                                       double headingOffsetDeg) {
        this.fieldOriginOffsetX = xOffsetInches;
        this.fieldOriginOffsetY = yOffsetInches;
        this.headingOffsetRad   = Math.toRadians(headingOffsetDeg);
    }

    /**
     * Mirror the Y axis (enable if the Limelight Y direction is inverted relative to Pedro).
     * Applied before the Y offset.
     */
    public void setMirrorY(boolean mirror) {
        this.mirrorY = mirror;
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
     * Returns the camera pose in Pedro field coordinates after Stage 1 only
     * (field origin + heading offsets applied; camera-to-robot-centre NOT applied).
     *
     * Useful for comparing raw Limelight output against the robot-centre estimate
     * or for verifying the field-origin calibration.
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

        // Stage 1: unit conversion + field-origin offset
        double xInches = botpose.getPosition().toUnit(DistanceUnit.INCH).x + fieldOriginOffsetX;
        double rawY    = botpose.getPosition().toUnit(DistanceUnit.INCH).y;
        double yInches = (mirrorY ? -rawY : rawY) + fieldOriginOffsetY;
        double cameraHeading = botpose.getOrientation().getYaw(AngleUnit.RADIANS) + headingOffsetRad;

        return new Pose(xInches, yInches, cameraHeading);
    }

    /**
     * Returns the latest robot-centre pose in Pedro Pathing coordinates (inches, radians).
     *
     * Applies both transform stages:
     *   1. Field-to-Pedro (unit conversion + coordinate offsets).
     *   2. Camera-mount-to-robot-centre (physical offset geometry).
     *
     * The camera faces backward (CAMERA_HEADING_OFFSET = π), so:
     *   robot_heading = camera_heading − π
     *
     * The camera mount offset (CAMERA_FORWARD_OFFSET, CAMERA_LEFT_OFFSET) is rotated
     * into the field frame at robot_heading and subtracted from the camera position to
     * give the robot centre.
     *
     * Returns null when no valid pose is available.
     */
    public Pose getRobotPose() {
        Pose cameraPose = getCameraRawPose();
        if (cameraPose == null) return null;

        // Stage 2a: derive robot heading from camera heading
        double robotHeading = normalizeAngle(cameraPose.getHeading() - CAMERA_HEADING_OFFSET);

        // Stage 2b: rotate camera mount offset (robot frame) into field frame
        double cam_dx = CAMERA_FORWARD_OFFSET * Math.cos(robotHeading)
                      - CAMERA_LEFT_OFFSET    * Math.sin(robotHeading);
        double cam_dy = CAMERA_FORWARD_OFFSET * Math.sin(robotHeading)
                      + CAMERA_LEFT_OFFSET    * Math.cos(robotHeading);

        // Stage 2c: robot centre = camera position − rotated offset
        double robotX = cameraPose.getX() - cam_dx;
        double robotY = cameraPose.getY() - cam_dy;

        lastValidPose = new Pose(robotX, robotY, robotHeading);
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
