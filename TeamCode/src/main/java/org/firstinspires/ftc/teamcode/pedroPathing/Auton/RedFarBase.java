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


@Autonomous(name = "RedFarBase", group = "RedAuton")
public class RedFarBase extends OpMode {

    private enum PathState {
        SCORE_PRELOAD,
        WAIT_SCORE_PRELOAD_COMPLETE,
        WAIT_SHOT_PRELOAD,
        RUN_ROW_CYCLE,
        WAIT_ROW_CYCLE_COMPLETE,
        WAIT_SHOT_AFTER_ROW,
        RUN_CORNER_CYCLE,
        WAIT_CORNER_CYCLE_COMPLETE,
        WAIT_SHOT_AFTER_CORNER,
        RUN_PARK,
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
    private static final int CORNER_CYCLE_REPEATS = 2;

    private final Pose startPose = new Pose(85.744, 7.484, Math.toRadians(-90));
    private final Pose preloadScorePose = new Pose(83.853, 14.509, Math.toRadians(-118));
    private final Pose rowPickupStartPose = new Pose(101.426, 34.583, Math.toRadians(0));
    private final Pose rowPickupEndPose = new Pose(119.619, 34.403, Math.toRadians(0));
    private final Pose scoreAfterRowPose = new Pose(83.699, 14.609, Math.toRadians(-118));
    private final Pose cornerPickupStartPose = new Pose(126.891, 8.423);
    private final Pose cornerPickupEndPose = new Pose(137.601, 8.258);
    private final Pose scoreAfterCornerPose = new Pose(84.054, 14.841, Math.toRadians(-118));
    private final Pose parkPose = new Pose(110.380, 14.098);

    private PathChain preloadToScore, rowCycle, firstCornerCycle, repeatCornerCycle, parkPath;
    private Timer pathTimer, opmodeTimer;
    private PathState pathState = PathState.FINISHED;
    private long shootSequenceStartMs;
    private long shootSequenceEndMs;
    private int completedCornerCycles;

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

    private void startCornerCycleOrPark() {
        if (completedCornerCycles < CORNER_CYCLE_REPEATS) {
            PathChain cyclePath = (completedCornerCycles == 0) ? firstCornerCycle : repeatCornerCycle;
            follower.followPath(cyclePath, true);
            setPathState(PathState.WAIT_CORNER_CYCLE_COMPLETE);
            return;
        }
        follower.followPath(parkPath, true);
        setPathState(PathState.WAIT_PARK_COMPLETE);
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
        return state == PathState.WAIT_SHOT_PRELOAD
            || state == PathState.WAIT_SHOT_AFTER_ROW
            || state == PathState.WAIT_SHOT_AFTER_CORNER;
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
    preloadToScore = follower.pathBuilder()
            .addPath(new BezierLine(startPose, preloadScorePose))
            .setLinearHeadingInterpolation(Math.toRadians(-90), Math.toRadians(-118))
            .build();

    rowCycle = follower.pathBuilder()
            .addPath(new BezierLine(preloadScorePose, rowPickupStartPose))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
            .addPath(new BezierLine(rowPickupStartPose, rowPickupEndPose))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(rowPickupEndPose, scoreAfterRowPose))
            .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-118))
            .build();

    firstCornerCycle = follower.pathBuilder()
            .addPath(new BezierLine(scoreAfterRowPose, cornerPickupStartPose))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(cornerPickupStartPose, cornerPickupEndPose))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(cornerPickupEndPose, scoreAfterCornerPose))
            .setTangentHeadingInterpolation()
            .build();

    repeatCornerCycle = follower.pathBuilder()
            .addPath(new BezierLine(scoreAfterCornerPose, cornerPickupStartPose))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(cornerPickupStartPose, cornerPickupEndPose))
            .setTangentHeadingInterpolation()
            .addPath(new BezierLine(cornerPickupEndPose, scoreAfterCornerPose))
            .setTangentHeadingInterpolation()
            .build();

    parkPath = follower.pathBuilder()
            .addPath(new BezierLine(scoreAfterCornerPose, parkPose))
            .setTangentHeadingInterpolation()
            .build();
}

    public void autonomousPathUpdate() {
    switch (pathState) {
        case SCORE_PRELOAD:
            follower.followPath(preloadToScore, true);
            setPathState(PathState.WAIT_SCORE_PRELOAD_COMPLETE);
            break;
        case WAIT_SCORE_PRELOAD_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_PRELOAD);
            }
            break;
        case WAIT_SHOT_PRELOAD:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(PathState.RUN_ROW_CYCLE);
            }
            break;
        case RUN_ROW_CYCLE:
            if(pathDelayElapsed()) {
                followPathAfterDelay(rowCycle, PathState.WAIT_ROW_CYCLE_COMPLETE);
            }
            break;
        case WAIT_ROW_CYCLE_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_AFTER_ROW);
            }
            break;
        case WAIT_SHOT_AFTER_ROW:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(PathState.RUN_CORNER_CYCLE);
            }
            break;
        case RUN_CORNER_CYCLE:
            if(pathDelayElapsed()) {
                startCornerCycleOrPark();
            }
            break;
        case WAIT_CORNER_CYCLE_COMPLETE:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(PathState.WAIT_SHOT_AFTER_CORNER);
            }
            break;
        case WAIT_SHOT_AFTER_CORNER:
            if (shootWindowElapsed()) {
                completedCornerCycles++;
                shootSequencer.startIntake();
                if (completedCornerCycles < CORNER_CYCLE_REPEATS) {
                    setPathState(PathState.RUN_CORNER_CYCLE);
                } else {
                    setPathState(PathState.RUN_PARK);
                }
            }
            break;
        case RUN_PARK:
            if(pathDelayElapsed()) {
                followPathAfterDelay(parkPath, PathState.WAIT_PARK_COMPLETE);
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
        completedCornerCycles = 0;
    }

    @Override
    public void init_loop() {}

    @Override
    public void start() {
        opmodeTimer.resetTimer();
        completedCornerCycles = 0;
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
        telemetry.addData("corner cycles", completedCornerCycles + "/" + CORNER_CYCLE_REPEATS);
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