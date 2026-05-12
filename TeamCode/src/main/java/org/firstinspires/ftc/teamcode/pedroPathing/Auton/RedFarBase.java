package org.firstinspires.ftc.teamcode.pedroPathing.Auton; // make sure this aligns with class location

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.pedroPathing.controller.ShootSequencer;
import org.firstinspires.ftc.teamcode.pedroPathing.CommonDefs.PositionDefs;


@Autonomous(name = "RedFarBase", group = "RedAuton")
public class RedFarBase extends OpMode {

    private enum PathState {
        SCORE_PRELOAD,
        WAIT_SCORE_PRELOAD_COMPLETE,
        WAIT_SHOT_1,
        DELAY_BEFORE_GRAB_1,
        WAIT_GRAB_1_COMPLETE,
        DELAY_BEFORE_SCORE_1,
        WAIT_SCORE_1_COMPLETE,
        WAIT_SHOT_2,
        DELAY_BEFORE_GRAB_2,
        WAIT_GRAB_2_COMPLETE,
        DELAY_BEFORE_SCORE_2,
        WAIT_SCORE_2_COMPLETE,
        WAIT_SHOT_3,
        DELAY_BEFORE_GRAB_3,
        WAIT_GRAB_3_COMPLETE,
        DELAY_BEFORE_SCORE_3,
        WAIT_SCORE_3_COMPLETE,
        WAIT_SHOT_4,
        DELAY_BEFORE_PARK,
        WAIT_PARK_COMPLETE,
        FINISHED
    }

    private Follower follower;
    private ShootSequencer shootSequencer;
    
    private static final boolean Enable_Debug_Path_Wait = false;
    private static final double Debug_Delay = 1000.0;

    private static final int SHOOT_DURATION_MS = 900;
    private static final double SHOOT_TARGET_VELOCITY = 3000.0;
    private static final double SHOOT_VELOCITY_THRESHOLD = 50.0;
    private static final long SHOOT_SEQUENCE_WAIT_MS = 4500;

    private final Pose startPose = new Pose(PositionDefs.START_X, PositionDefs.START_Y, PositionDefs.START_HEADING); // Blue-side start at the small triangle, facing up-field.
    private final Pose scorePose = new Pose(PositionDefs.SCORE_X, PositionDefs.SCORE_Y, PositionDefs.SCORE_HEADING); // Shooting pose.
    private final Pose pickup1StartPose = new Pose(PositionDefs.PICKUP1_START_X, PositionDefs.PICKUP1_START_Y, PositionDefs.PICKUP1_START_HEADING); // First row approach.
    private final Pose pickup1EndPose = new Pose(PositionDefs.PICKUP1_END_X, PositionDefs.PICKUP1_END_Y, PositionDefs.PICKUP1_END_HEADING); // First row pickup end.
    private final Pose pickup2StartPose = new Pose(PositionDefs.PICKUP2_START_X, PositionDefs.PICKUP2_START_Y, PositionDefs.PICKUP2_START_HEADING); // Second row approach.
    private final Pose pickup2EndPose = new Pose(PositionDefs.PICKUP2_END_X, PositionDefs.PICKUP2_END_Y, PositionDefs.PICKUP2_END_HEADING); // Second row pickup end.
    private final Pose pickup3StartPose = new Pose(PositionDefs.PICKUP3_START_X, PositionDefs.PICKUP3_START_Y, PositionDefs.PICKUP3_START_HEADING); // Third row approach.
    private final Pose pickup3EndPose = new Pose(PositionDefs.PICKUP3_END_X, PositionDefs.PICKUP3_END_Y, PositionDefs.PICKUP3_END_HEADING); // Third row pickup end.
    private final Pose parkPose = new Pose(PositionDefs.PARK_X, PositionDefs.PARK_Y, PositionDefs.PARK_HEADING); // Final parking pose.

    private Path scorePreload;
    private PathChain grabPickup1, scorePickup1, grabPickup2, scorePickup2, grabPickup3, scorePickup3, park;
    private Timer pathTimer, opmodeTimer;
    private PathState pathState = PathState.FINISHED;
    private long shootSequenceStartMs;
    private long shootSequenceEndMs;

    private boolean pathDelayElapsed() {
        if (!Enable_Debug_Path_Wait) {
            return true;
        }
        return pathTimer.getElapsedTime() >= Debug_Delay;
    }

    private void followPathAfterDelay(PathChain path, PathState nextState) {
        follower.followPath(path, true);
        setPathState(nextState);
    }

    private void beginTimedShootAtScorePose(PathState nextStateAfterShot) {
        shootSequencer.stopIntake();
        shootSequencer.startShootingSequence(SHOOT_DURATION_MS, SHOOT_TARGET_VELOCITY, SHOOT_VELOCITY_THRESHOLD);
        shootSequenceStartMs = System.currentTimeMillis();
        shootSequenceEndMs = System.currentTimeMillis() + SHOOT_SEQUENCE_WAIT_MS;
        setPathState(nextStateAfterShot);
    }

    private boolean shootWindowElapsed() {
        return System.currentTimeMillis() >= shootSequenceEndMs;
    }

    private boolean isShootingWindowState(PathState state) {
        return state == PathState.WAIT_SHOT_1
                || state == PathState.WAIT_SHOT_2
                || state == PathState.WAIT_SHOT_3
                || state == PathState.WAIT_SHOT_4;
    }

    private long getShootingElapsedMs() {
        if (!isShootingWindowState(pathState)) {
            return 0;
        }
        return Math.min(SHOOT_SEQUENCE_WAIT_MS, Math.max(0, System.currentTimeMillis() - shootSequenceStartMs));
    }

    private long getShootingRemainingMs() {
        if (!isShootingWindowState(pathState)) {
            return 0;
        }
        return Math.max(0, shootSequenceEndMs - System.currentTimeMillis());
    }

public void buildPaths() {
    /* This is our scorePreload path. We are using a BezierLine, which is a straight line. */
    scorePreload = new Path(new BezierLine(startPose, scorePose));
    scorePreload.setLinearHeadingInterpolation(startPose.getHeading(), scorePose.getHeading());

    /* Here is an example for Constant Interpolation
    scorePreload.setConstantInterpolation(startPose.getHeading()); */

    /* This is our grabPickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup1 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup1StartPose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup1StartPose.getHeading())
            .addPath(new BezierLine(pickup1StartPose, pickup1EndPose))
            .setLinearHeadingInterpolation(pickup1StartPose.getHeading(), pickup1EndPose.getHeading())
            .build();

    /* This is our scorePickup1 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup1 = follower.pathBuilder()
            .addPath(new BezierLine(pickup1EndPose, scorePose))
            .setLinearHeadingInterpolation(pickup1EndPose.getHeading(), scorePose.getHeading())
            .build();

    /* This is our grabPickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup2 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup2StartPose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup2StartPose.getHeading())
            .addPath(new BezierLine(pickup2StartPose, pickup2EndPose))
            .setLinearHeadingInterpolation(pickup2StartPose.getHeading(), pickup2EndPose.getHeading())
            .build();

    /* This is our scorePickup2 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup2 = follower.pathBuilder()
            .addPath(new BezierLine(pickup2EndPose, scorePose))
            .setLinearHeadingInterpolation(pickup2EndPose.getHeading(), scorePose.getHeading())
            .build();

    /* This is our grabPickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        grabPickup3 = follower.pathBuilder()
            .addPath(new BezierLine(scorePose, pickup3StartPose))
            .setLinearHeadingInterpolation(scorePose.getHeading(), pickup3StartPose.getHeading())
            .addPath(new BezierLine(pickup3StartPose, pickup3EndPose))
            .setLinearHeadingInterpolation(pickup3StartPose.getHeading(), pickup3EndPose.getHeading())
            .build();

    /* This is our scorePickup3 PathChain. We are using a single path with a BezierLine, which is a straight line. */
        scorePickup3 = follower.pathBuilder()
            .addPath(new BezierLine(pickup3EndPose, scorePose))
            .setLinearHeadingInterpolation(pickup3EndPose.getHeading(), scorePose.getHeading())
            .build();

            park = follower.pathBuilder()
                .addPath(new BezierLine(scorePose, parkPose))
                .setLinearHeadingInterpolation(scorePose.getHeading(), parkPose.getHeading())
                .build();
}

    public void autonomousPathUpdate() {
    switch (pathState) {
        case SCORE_PRELOAD:
            follower.followPath(scorePreload);
            setPathState(PathState.WAIT_SCORE_PRELOAD_COMPLETE);
            break;
        case WAIT_SCORE_PRELOAD_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_1);
            }
            break;
        case WAIT_SHOT_1:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(PathState.DELAY_BEFORE_GRAB_1);
            }
            break;
        case DELAY_BEFORE_GRAB_1:
            if(pathDelayElapsed()) {
                followPathAfterDelay(grabPickup1, PathState.WAIT_GRAB_1_COMPLETE);
            }
            break;
        case WAIT_GRAB_1_COMPLETE:
            if(!follower.isBusy()) {
                setPathState(PathState.DELAY_BEFORE_SCORE_1);
            }
            break;
        case DELAY_BEFORE_SCORE_1:
            if(pathDelayElapsed()) {
                shootSequencer.stopIntake();
                followPathAfterDelay(scorePickup1, PathState.WAIT_SCORE_1_COMPLETE);
            }
            break;
        case WAIT_SCORE_1_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_2);
            }
            break;
        case WAIT_SHOT_2:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(PathState.DELAY_BEFORE_GRAB_2);
            }
            break;
        case DELAY_BEFORE_GRAB_2:
            if(pathDelayElapsed()) {
                followPathAfterDelay(grabPickup2, PathState.WAIT_GRAB_2_COMPLETE);
            }
            break;
        case WAIT_GRAB_2_COMPLETE:
            if(!follower.isBusy()) {
                setPathState(PathState.DELAY_BEFORE_SCORE_2);
            }
            break;
        case DELAY_BEFORE_SCORE_2:
            if(pathDelayElapsed()) {
                shootSequencer.stopIntake();
                followPathAfterDelay(scorePickup2, PathState.WAIT_SCORE_2_COMPLETE);
            }
            break;
        case WAIT_SCORE_2_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_3);
            }
            break;
        case WAIT_SHOT_3:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(PathState.DELAY_BEFORE_GRAB_3);
            }
            break;
        case DELAY_BEFORE_GRAB_3:
            if(pathDelayElapsed()) {
                followPathAfterDelay(grabPickup3, PathState.WAIT_GRAB_3_COMPLETE);
            }
            break;
        case WAIT_GRAB_3_COMPLETE:
            if(!follower.isBusy()) {
                setPathState(PathState.DELAY_BEFORE_SCORE_3);
            }
            break;
        case DELAY_BEFORE_SCORE_3:
            if(pathDelayElapsed()) {
                shootSequencer.stopIntake();
                followPathAfterDelay(scorePickup3, PathState.WAIT_SCORE_3_COMPLETE);
            }
            break;
        case WAIT_SCORE_3_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_4);
            }
            break;
        case WAIT_SHOT_4:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(PathState.DELAY_BEFORE_PARK);
            }
            break;
        case DELAY_BEFORE_PARK:
            if(pathDelayElapsed()) {
                followPathAfterDelay(park, PathState.WAIT_PARK_COMPLETE);
            }
            break;
        case WAIT_PARK_COMPLETE:
            if(!follower.isBusy()) {
                shootSequencer.stopIntake();
                shootSequencer.setShooterRunningAfterShoot(false);
                shootSequencer.stopShootingSequence();
                setPathState(PathState.FINISHED);
            }
            break;
        case FINISHED:
            break;
    }
}

/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
public void setPathState(PathState pState) {
    pathState = pState;
    pathTimer.resetTimer();
}

    @Override
    public void init() {
        pathTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();

        shootSequencer = new ShootSequencer();
        shootSequencer.init(
                hardwareMap
        );
            shootSequencer.setShooterRunningAfterShoot(true);

        follower = Constants.createFollower(hardwareMap);
        buildPaths();
        follower.setStartingPose(startPose);
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(PathState.SCORE_PRELOAD);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        shootSequencer.loop(false);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
        telemetry.addData("shoot elapsed ms", getShootingElapsedMs());
        telemetry.addData("shoot remaining ms", getShootingRemainingMs());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", follower.getPose().getHeading());
        telemetry.update();
    }

    @Override
    public void stop() {
        shootSequencer.setShooterRunningAfterShoot(false);
        shootSequencer.stopIntake();
        shootSequencer.stopShootingSequence();
    }


}