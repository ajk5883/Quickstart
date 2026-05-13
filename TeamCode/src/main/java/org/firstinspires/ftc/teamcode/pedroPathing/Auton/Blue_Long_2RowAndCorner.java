package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import java.util.ArrayList;
import java.util.List;

/**
 * Blue alliance – Long side auton: 1 row pickup + 2 corner cycles, then park.
 *
 * All poses are point-symmetric mirrors of the Red Long poses across the
 * field centre (144 in × 144 in field):
 *   x_blue = 144 − x_red,  y_blue = 144 − y_red,  θ_blue = θ_red + π
 *
 * Field origin: Pedro Pathing (0, 0) at the near-blue corner.
 * Robot faces away from the field wall (intake forward / shooter + camera backward).
 *
 * Identical to {@link Blue_Long_1RowAndCorner} except {@code cornerCycles = 2}.
 */
@Autonomous(name = "Blue L 2R", group = "BlueAuton")
public class Blue_Long_2RowAndCorner extends AutonBase {

    @Override
    protected AutonConfig getConfig() {

        // ── Poses ──────────────────────────────────────────────────────────────
        final Pose startPose             = new Pose(58.256, 136.516, Math.toRadians(90));
        final Pose preloadScorePose      = new Pose(60.147, 129.491, Math.toRadians(62));
        final Pose rowPickupStartPose    = new Pose(42.574, 109.417, Math.toRadians(180));
        final Pose rowPickupEndPose      = new Pose(24.381, 109.597, Math.toRadians(180));
        final Pose rowScorePose          = new Pose(60.301, 129.391, Math.toRadians(62));
        final Pose cornerPickupStartPose = new Pose(17.109, 135.577);
        final Pose cornerPickupEndPose   = new Pose(6.399,  135.742);
        final Pose cornerScorePose       = new Pose(59.946, 129.159, Math.toRadians(62));
        final Pose parkPose              = new Pose(33.620, 129.902);
        final int  cornerCycles          = 2;

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
