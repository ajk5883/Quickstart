package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

/**
 * Red alliance – Long side auton: 1 row pickup + 1 corner cycle, then park.
 *
 * Field origin: Pedro Pathing (0, 0) at the near-red corner.
 * Robot faces away from the field wall (intake forward / shooter + camera backward).
 *
 * Step sequence
 * ─────────────
 *   STARTING      – (no path) shooter kept running between shots
 *   SCORING       – start → preload score pose
 *   PICKUP_START  – preload score → row pickup start  (intake on)
 *   PICKUP_END    – row pickup start → row pickup end  (intake off)
 *   SCORING       – row pickup end → row score pose
 *   PICKUP_START  – row score → corner pickup start  (intake on)
 *   PICKUP_END    – corner pickup start → corner pickup end  (intake off)
 *   SCORING       – corner pickup end → corner score pose
 *   PARKING       – corner score → park pose  (all stopped)
 */
@Autonomous(name = "Red L 1R", group = "RedAuton")
public class Red_Long_1RowAndCorner extends AutonBase {

    @Override
    protected AutonConfig getConfig() {

        // ── Poses ──────────────────────────────────────────────────────────────
        final Pose startPose             = new Pose(85.744,  7.484,  Math.toRadians(-90));
        final Pose preloadScorePose      = new Pose(83.853,  14.509, Math.toRadians(-118));
        final Pose rowPickupStartPose    = new Pose(101.426, 34.583, Math.toRadians(0));
        final Pose rowPickupEndPose      = new Pose(119.619, 34.403, Math.toRadians(0));
        final Pose rowScorePose          = new Pose(83.699,  14.609, Math.toRadians(-118));
        final Pose cornerPickupStartPose = new Pose(126.891, 8.423);
        final Pose cornerPickupEndPose   = new Pose(137.601, 8.258);
        final Pose cornerScorePose       = new Pose(84.054,  14.841, Math.toRadians(-118));
        final Pose parkPose              = new Pose(110.380, 14.098);
        final int  cornerCycles          = 1;

        return AutonConfig.withDefaults(startPose, follower -> {
            List<PathStep> steps = new ArrayList<>();

            // ── STARTING ──────────────────────────────────────────────────────
            steps.add(new PathStep(PoseType.STARTING, null));

            // ── Shoot parameters (per scoring pose) ──────────────────────────
            // Tune each scoring pose's RPM and hood position independently.
            final double preloadRpm  = 3000.0; final double preloadHood  = 0.35;
            final double rowRpm      = 3000.0; final double rowHood      = 0.35;
            final double cornerRpm   = 3000.0; final double cornerHood   = 0.35;

            // ── Preload: start → score ────────────────────────────────────────
            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(startPose, preloadScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                    .build(), preloadRpm, preloadHood));

            // ── Row pickup sweep ──────────────────────────────────────────────
            steps.add(new PathStep(PoseType.PICKUP_START, follower.pathBuilder()
                    .addPath(new BezierLine(preloadScorePose, rowPickupStartPose))
                    .setLinearHeadingInterpolation(
                            rowPickupStartPose.getHeading(),
                            rowPickupStartPose.getHeading())
                    .build()));

            steps.add(new PathStep(PoseType.PICKUP_END, follower.pathBuilder()
                    .addPath(new BezierLine(rowPickupStartPose, rowPickupEndPose))
                    .setTangentHeadingInterpolation()
                    .build()));

            // ── Row score ─────────────────────────────────────────────────────
            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(rowPickupEndPose, rowScorePose))
                    .setLinearHeadingInterpolation(rowPickupEndPose.getHeading(), rowScorePose.getHeading())
                    .build(), rowRpm, rowHood));

            // ── Corner cycles ─────────────────────────────────────────────────
            Pose prevScore = rowScorePose;
            for (int i = 0; i < cornerCycles; i++) {

                steps.add(new PathStep(PoseType.PICKUP_START, follower.pathBuilder()
                        .addPath(new BezierLine(prevScore, cornerPickupStartPose))
                        .setTangentHeadingInterpolation()
                        .build()));

                steps.add(new PathStep(PoseType.PICKUP_END, follower.pathBuilder()
                        .addPath(new BezierLine(cornerPickupStartPose, cornerPickupEndPose))
                        .setTangentHeadingInterpolation()
                        .build()));

                steps.add(PathStep.scoring(follower.pathBuilder()
                        .addPath(new BezierLine(cornerPickupEndPose, cornerScorePose))
                        .setTangentHeadingInterpolation()
                        .build(), cornerRpm, cornerHood));

                prevScore = cornerScorePose;
            }

            // ── Park ───────────────────────────────────────────────────────────
            steps.add(new PathStep(PoseType.PARKING, follower.pathBuilder()
                    .addPath(new BezierLine(prevScore, parkPose))
                    .setTangentHeadingInterpolation()
                    .build()));

            return steps;
        });
    }
}
