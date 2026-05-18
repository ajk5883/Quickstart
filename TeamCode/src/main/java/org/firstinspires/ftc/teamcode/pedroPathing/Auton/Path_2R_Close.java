package org.firstinspires.ftc.teamcode.pedroPathing.Auton;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig;
import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.ParamsConfig.Alliance;

import java.util.ArrayList;
import java.util.List;

public class Path_2R_Close {

    protected AutonConfig getConfig(Alliance alliance) {
        final Pose startPose           = alliance == Alliance.RED ? ParamsConfig.POSE_START_RED_CLOSE : ParamsConfig.POSE_START_BLUE_CLOSE;
        final Pose preloadScorePose    = alliance == Alliance.RED ? ParamsConfig.POSE_SCORE_RED_CLOSE : ParamsConfig.POSE_SCORE_BLUE_CLOSE;
        final Pose row3PickupStartPose = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW3_START_RED : ParamsConfig.POSE_BALLS_ROW3_START_BLUE;
        final Pose row3PickupEndPose   = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW3_END_RED : ParamsConfig.POSE_BALLS_ROW3_END_BLUE;
        final Pose row2PickupStartPose = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW2_START_RED_CLOSE : ParamsConfig.POSE_BALLS_ROW2_START_BLUE_CLOSE;
        final Pose row2PickupEndPose   = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW2_END_RED_CLOSE : ParamsConfig.POSE_BALLS_ROW2_END_BLUE_CLOSE;
        final Pose scorePose           = alliance == Alliance.RED ? ParamsConfig.POSE_SCORE_RED_CLOSE : ParamsConfig.POSE_SCORE_BLUE_CLOSE;
        final Pose parkPose            = alliance == Alliance.RED ? ParamsConfig.POSE_PARK_RED_CLOSE : ParamsConfig.POSE_PARK_BLUE_CLOSE;

        return AutonConfig.withCloseDefaults(startPose, follower -> {
            List<PathStep> steps = new ArrayList<>();

            steps.add(new PathStep(PoseType.STARTING, null));

            final double preloadRpm = ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_CLOSE;
            final double preloadHood = ParamsConfig.AUTON_SHOOT_HOOD_POSITION_CLOSE;
            final double rowRpm = ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_CLOSE;
            final double rowHood = ParamsConfig.AUTON_SHOOT_HOOD_POSITION_CLOSE;

            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(startPose, preloadScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                    .build(), preloadRpm, preloadHood, 1.0));

            // Pickup row 3 first
            steps.add(new PathStep(PoseType.PICKUP_START, follower.pathBuilder()
                    .addPath(new BezierLine(preloadScorePose, row3PickupStartPose))
                    .setLinearHeadingInterpolation(preloadScorePose.getHeading(), row3PickupStartPose.getHeading(), 0.5)
                    .build()));
            steps.set(steps.size() - 1, new PathStep(PoseType.PICKUP_START, steps.get(steps.size() - 1).path, 1.0));

            steps.add(new PathStep(PoseType.PICKUP_END, follower.pathBuilder()
                    .addPath(new BezierLine(row3PickupStartPose, row3PickupEndPose))
                    .setTangentHeadingInterpolation()
                    .build()));
            steps.set(steps.size() - 1, new PathStep(PoseType.PICKUP_END, steps.get(steps.size() - 1).path, 0.5));

            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(row3PickupEndPose, scorePose))
                    .setLinearHeadingInterpolation(row3PickupEndPose.getHeading(), scorePose.getHeading())
                    .build(), rowRpm, rowHood, 1.0));

            steps.add(new PathStep(PoseType.PICKUP_START, follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, row2PickupStartPose))
                    .setLinearHeadingInterpolation(scorePose.getHeading(), row2PickupStartPose.getHeading())
                    .build()));
            steps.set(steps.size() - 1, new PathStep(PoseType.PICKUP_START, steps.get(steps.size() - 1).path, 1.0));

            steps.add(new PathStep(PoseType.PICKUP_END, follower.pathBuilder()
                    .addPath(new BezierLine(row2PickupStartPose, row2PickupEndPose))
                    .setTangentHeadingInterpolation()
                    .build()));
            steps.set(steps.size() - 1, new PathStep(PoseType.PICKUP_END, steps.get(steps.size() - 1).path, 0.5));

            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(row2PickupEndPose, scorePose))
                    .setLinearHeadingInterpolation(row2PickupEndPose.getHeading(), scorePose.getHeading())
                    .build(), rowRpm, rowHood, 1.0));

            steps.add(new PathStep(PoseType.PARKING, follower.pathBuilder()
                    .addPath(new BezierLine(scorePose, parkPose))
                    .setTangentHeadingInterpolation()
                    .build(), 1.0));

            return steps;
        });
    }
}