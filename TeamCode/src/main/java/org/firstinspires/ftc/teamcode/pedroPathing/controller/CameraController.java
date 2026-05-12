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
 * PIPELINE SETUP:
 *   Use Pipeline 0 — the Limelight's built-in AprilTag detection pipeline.
 *   This detects ALL field AprilTags simultaneously. Filter by tag ID in code
 *   to identify alliance-specific targets. Running a single pipeline also enables
 *   MegaTag averaged pose estimation, which is more accurate than single-tag pose.
 *
 * DECODE SEASON DEFAULTS (TU32):
 *   - Blue GOAL tag: 20
 *   - Red GOAL tag: 24
 *   - OBELISK tags: 21, 22, 33
 *
 *   For localization, OBELISK tags are excluded by default because placement can
 *   vary match-to-match. The controller only trusts frames that contain at least
 *   one GOAL tag unless decode filtering is disabled.
 *
 * COORDINATE TRANSFORM:
 *   The Limelight returns botpose in meters using its configured field origin.
 *   Pedro Pathing uses inches. Call setCoordinateTransform() to align the two
 *   coordinate systems once you know the offset between the Limelight field origin
 *   and Pedro's (0,0) corner.
 *
 * HARDWARE MAP NAME:
 *   Default device name is "limelight". Configure in the Driver Hub robot config.
 */
public class CameraController {

    /** Built-in AprilTag pipeline index on the Limelight 3A. Detects all tags. */
    public static final int APRILTAG_PIPELINE = 0;

    // FTC DECODE TU32 field tags
    public static final int DECODE_BLUE_GOAL_TAG_ID = 20;
    public static final int DECODE_RED_GOAL_TAG_ID = 24;
    public static final int DECODE_OBELISK_TAG_ID_1 = 21;
    public static final int DECODE_OBELISK_TAG_ID_2 = 22;
    public static final int DECODE_OBELISK_TAG_ID_3 = 33;

    private static final double METERS_TO_INCHES = 39.3701;

    // Botpose array indices returned by Limelight
    private static final int BOTPOSE_X_IDX       = 0;
    private static final int BOTPOSE_Y_IDX       = 1;
    private static final int BOTPOSE_YAW_IDX     = 5;
    private static final int BOTPOSE_LATENCY_IDX = 6;
    private static final int BOTPOSE_MIN_LENGTH  = 6;

    private Limelight3A limelight;
    private boolean isInitialized = false;

    // DECODE tag groups used for visibility checks and localization filtering
    private final Set<Integer> decodeGoalTagIds = new HashSet<>();
    private final Set<Integer> decodeObeliskTagIds = new HashSet<>();
    private boolean useDecodeGoalOnlyLocalization = true;

    // Coordinate transform: applied after meters → inches conversion
    private double fieldOriginOffsetX   = 0.0;  // inches
    private double fieldOriginOffsetY   = 0.0;  // inches
    private double headingOffsetRad     = 0.0;  // radians

    // Whether to mirror the Y axis (needed if Limelight Y is inverted relative to Pedro)
    private boolean mirrorY = false;

    private Pose lastValidPose = null;

        public CameraController() {
        setDecodeTagGroups(
            DECODE_BLUE_GOAL_TAG_ID,
            DECODE_RED_GOAL_TAG_ID,
            new int[] {
                DECODE_OBELISK_TAG_ID_1,
                DECODE_OBELISK_TAG_ID_2,
                DECODE_OBELISK_TAG_ID_3
            }
        );
        }

    // -------------------------------------------------------------------------
    //  Initialisation
    // -------------------------------------------------------------------------

    /**
     * Initialise with the default hardware map name "limelight" and Pipeline 0.
     */
    public void init(HardwareMap hardwareMap) {
        init(hardwareMap, "limelight", APRILTAG_PIPELINE);
    }

    /**
     * Initialise with a custom hardware map name and pipeline index.
     *
     * @param hardwareMap  OpMode hardware map
     * @param deviceName   Name configured in the Driver Hub robot config
     * @param pipeline     Pipeline index (0 = built-in AprilTag; keep at 0 for pose estimation)
     */
    public void init(HardwareMap hardwareMap, String deviceName, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, deviceName);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
        isInitialized = true;
    }

    // -------------------------------------------------------------------------
    //  Coordinate transform configuration
    // -------------------------------------------------------------------------

    /**
     * Set the offset between the Limelight's field coordinate origin and Pedro's (0,0).
     *
     * Example: if the Limelight field origin is at Pedro's (-72, -72) you would call
     *   setCoordinateTransform(72, 72, 0)
     *
     * @param xOffsetInches      X offset (inches) added after meters→inches conversion
     * @param yOffsetInches      Y offset (inches) added after meters→inches conversion
     * @param headingOffsetDeg   Heading offset (degrees) added to the Limelight yaw
     */
    public void setCoordinateTransform(double xOffsetInches,
                                       double yOffsetInches,
                                       double headingOffsetDeg) {
        this.fieldOriginOffsetX = xOffsetInches;
        this.fieldOriginOffsetY = yOffsetInches;
        this.headingOffsetRad  = Math.toRadians(headingOffsetDeg);
    }

    /**
     * Mirror the Y axis. Enable this if the Limelight Y direction is opposite to Pedro's Y.
     * The mirror is applied before the Y offset.
     *
     * @param mirror true to negate the Y value from the Limelight
     */
    public void setMirrorY(boolean mirror) {
        this.mirrorY = mirror;
    }

    // -------------------------------------------------------------------------
    //  Pose estimation
    // -------------------------------------------------------------------------

    /**
     * Returns the latest robot pose estimate from AprilTags in Pedro Pathing coordinates
     * (x inches, y inches, heading radians).
     *
     * Returns null when:
     *   - The camera is not initialised
     *   - No valid Limelight result is available
     *   - No AprilTags are detected
     *   - The botpose array is malformed
     */
    public Pose getRobotPose() {
        if (!isInitialized) return null;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;

        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return null;
        if (useDecodeGoalOnlyLocalization && !containsAnyTag(fiducials, decodeGoalTagIds)) return null;

        // NEW: Get the Pose3D object instead of double[]
        Pose3D botpose = result.getBotpose();
        if (botpose == null) return null;

        // Convert the values directly using the Pose3D methods
        double xInches = (botpose.getPosition().toUnit(DistanceUnit.INCH).x) + fieldOriginOffsetX;
        double rawY    = botpose.getPosition().toUnit(DistanceUnit.INCH).y;
        double yInches = (mirrorY ? -rawY : rawY) + fieldOriginOffsetY;

        // Pedro Pathing typically uses Radians for heading
        double heading = botpose.getOrientation().getYaw(AngleUnit.RADIANS) + headingOffsetRad;

        lastValidPose = new Pose(xInches, yInches, heading);
        return lastValidPose;
    }

    /**
     * Enable/disable DECODE localization filtering.
     *
     * When enabled (default), pose is returned only if at least one GOAL tag is visible.
     * OBELISK-only frames are ignored for localization stability.
     */
    public void setUseDecodeGoalOnlyLocalization(boolean enabled) {
        this.useDecodeGoalOnlyLocalization = enabled;
    }

    public boolean isUsingDecodeGoalOnlyLocalization() {
        return useDecodeGoalOnlyLocalization;
    }

    /**
     * Override DECODE tag groups if FIRST updates IDs in a future Team Update.
     */
    public void setDecodeTagGroups(int blueGoalTagId, int redGoalTagId, int[] obeliskTagIds) {
        decodeGoalTagIds.clear();
        decodeGoalTagIds.add(blueGoalTagId);
        decodeGoalTagIds.add(redGoalTagId);

        decodeObeliskTagIds.clear();
        if (obeliskTagIds != null) {
            for (int id : obeliskTagIds) {
                decodeObeliskTagIds.add(id);
            }
        }
    }

    /**
     * Returns the last successfully computed pose. May be stale if tags are no longer visible.
     * Check {@link #getTagCount()} before trusting this for navigation.
     */
    public Pose getLastValidPose() {
        return lastValidPose;
    }

    /**
     * Returns true if a fresh valid pose estimate is available this cycle.
     */
    public boolean hasPoseEstimate() {
        return getRobotPose() != null;
    }

    // -------------------------------------------------------------------------
    //  Tag inspection
    // -------------------------------------------------------------------------

    /**
     * Returns the number of AprilTags detected in the latest frame.
     */
    public int getTagCount() {
        if (!isInitialized) return 0;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return 0;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        return (fiducials == null) ? 0 : fiducials.size();
    }

    /**
     * Returns the raw list of fiducial (AprilTag) results from the latest frame.
     * Use this to read individual tag IDs and poses when you need to identify
     * specific field elements (e.g. goal-post tags vs. alliance wall tags).
     *
     * Returns null if no result is available.
     */
    public List<LLResultTypes.FiducialResult> getFiducialResults() {
        if (!isInitialized) return null;
        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return null;
        return result.getFiducialResults();
    }

    /**
     * Returns true if the tag with the given ID is currently visible.
     *
     * @param tagId  AprilTag ID to search for
     */
    public boolean isTagVisible(int tagId) {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null) return false;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (f.getFiducialId() == tagId) return true;
        }
        return false;
    }

    /** Returns true if either DECODE GOAL tag (20 or 24 by default) is visible. */
    public boolean isAnyDecodeGoalTagVisible() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        return fiducials != null && containsAnyTag(fiducials, decodeGoalTagIds);
    }

    /** Returns true if at least one OBELISK tag is visible in the latest frame. */
    public boolean isAnyDecodeObeliskTagVisible() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        return fiducials != null && containsAnyTag(fiducials, decodeObeliskTagIds);
    }

    /** Returns true when the camera sees tags, but all visible tags are OBELISK tags. */
    public boolean isOnlyDecodeObeliskVisible() {
        List<LLResultTypes.FiducialResult> fiducials = getFiducialResults();
        if (fiducials == null || fiducials.isEmpty()) return false;
        return containsAnyTag(fiducials, decodeObeliskTagIds)
                && !containsAnyTag(fiducials, decodeGoalTagIds);
    }

    /**
     * Returns detection latency in milliseconds (index 6 of the botpose array).
     * Returns -1 if unavailable. Use this for latency-compensated pose fusion.
     */


    // -------------------------------------------------------------------------
    //  Lifecycle
    // -------------------------------------------------------------------------

    /**
     * Switch the active Limelight pipeline at runtime.
     * Keep at {@link #APRILTAG_PIPELINE} (0) for pose estimation.
     */
    public void setPipeline(int pipelineIndex) {
        if (limelight != null) {
            limelight.pipelineSwitch(pipelineIndex);
        }
    }

    /**
     * Stop the Limelight to reduce power/CPU overhead (e.g. end of OpMode).
     */
    public void stop() {
        if (limelight != null) {
            limelight.stop();
        }
    }

    public boolean isInitialized() {
        return isInitialized;
    }

    private boolean containsAnyTag(List<LLResultTypes.FiducialResult> fiducials, Set<Integer> ids) {
        if (ids.isEmpty()) return true;
        for (LLResultTypes.FiducialResult f : fiducials) {
            if (ids.contains(f.getFiducialId())) return true;
        }
        return false;
    }
}
