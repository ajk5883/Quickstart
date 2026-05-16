package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.CameraController;

@Configurable
public final class TeleOpTuningConfig {
    private TeleOpTuningConfig() {}

    // Shared drive and control tuning
    public static double STICK_SCALE = 1.0;
    public static double TRIGGER_THRESHOLD = 0.15;
    public static double SHOOTER_VELOCITY_THRESHOLD_RPM = 60.0;
    public static int ASSISTED_SHOOT_DURATION_MS = 900;
    public static double HEADING_ALIGN_KP = 1.2;
    public static double HEADING_ALIGN_TOLERANCE_RAD = Math.toRadians(2.0);
    public static double HEADING_ALIGN_MAX_TURN = 0.6;

    // Maximum RPM allowed for manual adjustments
    public static double MAX_MANUAL_RPM = 6000.0;

    // Manual target adjustment steps
    public static double CLOSE_RPM_STEP = 50.0;
    public static double FAR_RPM_STEP = 50.0;
    public static double CLOSE_HOOD_STEP_DEG = 0.1;
    public static double FAR_HOOD_STEP_DEG = 0.1;

    // Manual mode defaults
    public static double TARGET_MANUAL_CLOSE_RPM = 3200;
    public static double TARGET_MANUAL_CLOSE_HOOD_DEG = 0.0;
    public static double TARGET_MANUAL_FAR_RPM = 4800;
    public static double TARGET_MANUAL_FAR_HOOD_DEG = 0.7;

    public static class ShotConfig {
                public final double rpm;
                public final double hoodPosition;

                public ShotConfig(double rpm, double hoodPosition) {
                        this.rpm = rpm;
                        this.hoodPosition = hoodPosition;
                }
    }

    public static class AllianceTeleOpConfig {
        public final Pose startPose;
        public final Pose goalPose;
        public final Pose[] presetPoses;
        public final ShotConfig[] presetShots;
        /** Limelight pipeline index to activate for this alliance (see CameraController). */
        public final int cameraPipeline;

        public AllianceTeleOpConfig(Pose startPose, Pose goalPose, Pose[] presetPoses,
                                    ShotConfig[] presetShots, int cameraPipeline) {
            this.startPose = startPose;
            this.goalPose = goalPose;
            this.presetPoses = presetPoses;
            this.presetShots = presetShots;
            this.cameraPipeline = cameraPipeline;
        }
    }

    public static final AllianceTeleOpConfig RED = new AllianceTeleOpConfig(
            new Pose(85, 7, Math.toRadians(-90)),
            new Pose(144.0, 24.0, 0.0),
            new Pose[] {
                    new Pose(78, 77, Math.toRadians(-180+48)),
                    new Pose(87, 115, Math.toRadians(-180+25)),
                    new Pose(76, 132, Math.toRadians(-180)),
                    new Pose(79, 12, Math.toRadians(-180+67))
            },
            new ShotConfig[] {
                    new ShotConfig(3100, 0.3),
                    new ShotConfig(3100, 0.3),
                    new ShotConfig(3100, 0.3),
                    new ShotConfig(4800, 0.7)
            },
            CameraController.RED_GOAL_PIPELINE
    );

    public static final AllianceTeleOpConfig BLUE = new AllianceTeleOpConfig(
            new Pose(58.256, 136.516, Math.toRadians(90)),
            new Pose(0.0, 120.0, Math.PI),
            new Pose[] {
                    new Pose(60.147, 129.491, Math.toRadians(62)),
                    new Pose(60.301, 129.391, Math.toRadians(62)),
                    new Pose(59.946, 129.159, Math.toRadians(62)),
                    new Pose(33.620, 129.902, Math.toRadians(180))
            },
            new ShotConfig[] {
                    new ShotConfig(3000.0, 0.34),
                    new ShotConfig(3050.0, 0.35),
                    new ShotConfig(3100.0, 0.36),
                    new ShotConfig(3500.0, 0.44)
            },
            CameraController.BLUE_GOAL_PIPELINE
    );
}
