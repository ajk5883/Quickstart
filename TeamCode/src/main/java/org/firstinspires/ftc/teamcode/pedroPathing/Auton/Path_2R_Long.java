public class Path_2R_Long {
    
    @Override
    protected AutonConfig getConfig(Alliance alliance) {
        // ── Poses ──────────────────────────────────────────────────────────────
        final Pose startPose             = alliance == Alliance.RED ? ParamsConfig.POSE_START_RED_LONG : ParamsConfig.POSE_START_BLUE_LONG;
        final Pose preloadScorePose      = alliance == Alliance.RED ? ParamsConfig.POSE_SCORE_RED_LONG : ParamsConfig.POSE_SCORE_BLUE_LONG;
        final Pose row1PickupStartPose   = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW1_START_RED : ParamsConfig.POSE_BALLS_ROW1_START_BLUE;
        final Pose row1PickupEndPose     = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW1_END_RED : ParamsConfig.POSE_BALLS_ROW1_END_BLUE;
        final Pose row2PickupStartPose   = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW2_START_RED : ParamsConfig.POSE_BALLS_ROW2_START_BLUE;
        final Pose row2PickupEndPose     = alliance == Alliance.RED ? ParamsConfig.POSE_BALLS_ROW2_END_RED : ParamsConfig.POSE_BALLS_ROW2_END_BLUE;
        final Pose rowScorePose          = alliance == Alliance.RED ? ParamsConfig.POSE_SCORE_RED_LONG : ParamsConfig.POSE_SCORE_BLUE_LONG;
        final Pose cornerPickupStartPose = alliance == Alliance.RED ? ParamsConfig.POSE_CORNER_PICKUP_START_RED_LONG : ParamsConfig.POSE_CORNER_PICKUP_START_BLUE_LONG;
        final Pose cornerPickupEndPose   = alliance == Alliance.RED ? ParamsConfig.POSE_CORNER_PICKUP_RED_LONG : ParamsConfig.POSE_CORNER_PICKUP_BLUE_LONG;
        final Pose cornerScorePose       = alliance == Alliance.RED ? ParamsConfig.POSE_SCORE_RED_LONG : ParamsConfig.POSE_SCORE_BLUE_LONG;
        final Pose parkPose              = alliance == Alliance.RED ? ParamsConfig.POSE_PARK_RED_LONG : ParamsConfig.POSE_PARK_BLUE_LONG;
        final int  cornerCycles          = 2;

        return AutonConfig.withDefaults(startPose, follower -> {
            List<PathStep> steps = new ArrayList<>();

            // ── STARTING ──────────────────────────────────────────────────────
            steps.add(new PathStep(PoseType.STARTING, null));

            // ── Shoot parameters (per scoring pose) ──────────────────────────
            final double preloadRpm  = ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_FAR; final double preloadHood  = ParamsConfig.AUTON_SHOOT_HOOD_POSITION_FAR;
            final double rowRpm      = ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_FAR; final double rowHood      = ParamsConfig.AUTON_SHOOT_HOOD_POSITION_FAR;
            final double cornerRpm   = ParamsConfig.AUTON_SHOOT_TARGET_VELOCITY_FAR; final double cornerHood   = ParamsConfig.AUTON_SHOOT_HOOD_POSITION_FAR;

            // ── Preload: start → score ────────────────────────────────────────
            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(startPose, preloadScorePose))
                    .setLinearHeadingInterpolation(startPose.getHeading(), preloadScorePose.getHeading())
                    .build(), preloadRpm, preloadHood));

            // ── Row pickup sweep ──────────────────────────────────────────────
            steps.add(new PathStep(PoseType.PICKUP_START, follower.pathBuilder()
                    .addPath(new BezierLine(preloadScorePose, row1PickupStartPose))
                    .setLinearHeadingInterpolation(
                            preloadScorePose.getHeading(),
                            row1PickupStartPose.getHeading())
                    .build()));

            steps.add(new PathStep(PoseType.PICKUP_END, follower.pathBuilder()
                    .addPath(new BezierLine(row1PickupStartPose, row1PickupEndPose))
                    .setTangentHeadingInterpolation()
                    .build()));

            // ── Row score ─────────────────────────────────────────────────────
            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(row1PickupEndPose, rowScorePose))
                    .setLinearHeadingInterpolation(row1PickupEndPose.getHeading(), rowScorePose.getHeading())
                    .build(), rowRpm, rowHood));

            // ── Row 2 pickup + score ────────────────────────────────────────
            steps.add(new PathStep(PoseType.PICKUP_START, follower.pathBuilder()
                    .addPath(new BezierLine(rowScorePose, row2PickupStartPose))
                    .setLinearHeadingInterpolation(
                            rowScorePose.getHeading(),
                            row2PickupStartPose.getHeading())
                    .build()));

            steps.add(new PathStep(PoseType.PICKUP_END, follower.pathBuilder()
                    .addPath(new BezierLine(row2PickupStartPose, row2PickupEndPose))
                    .setTangentHeadingInterpolation()
                    .build()));

            steps.add(PathStep.scoring(follower.pathBuilder()
                    .addPath(new BezierLine(row2PickupEndPose, rowScorePose))
                    .setLinearHeadingInterpolation(row2PickupEndPose.getHeading(), rowScorePose.getHeading())
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
