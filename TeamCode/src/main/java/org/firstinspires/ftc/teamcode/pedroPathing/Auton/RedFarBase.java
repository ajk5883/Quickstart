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

    private Follower follower;
    private ShootSequencer shootSequencer;
    private static final double PATH_DELAY_MILLISECONDS = 5000.0;
    private static final String SHOOTER_MOTOR_1_NAME = "shooter1";
    private static final String SHOOTER_MOTOR_2_NAME = "shooter2";
    private static final String HOOD_SERVO_NAME = "hood";
    private static final double HOOD_MIN_POS = 0.0;
    private static final double HOOD_MAX_POS = 1.0;
    private static final String GATE_SERVO_NAME = "gate";
    private static final double GATE_OPEN_POS = 1.0;
    private static final double GATE_CLOSED_POS = 0.0;
    private static final String SPINNER_MOTOR_NAME = "spinner";
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
    private int pathState;
    private long shootSequenceEndMs;

    private boolean pathDelayElapsed() {
        return pathTimer.getElapsedTime() >= PATH_DELAY_MILLISECONDS;
    }

    private void followPathAfterDelay(PathChain path, int nextState) {
        follower.followPath(path, true);
        setPathState(nextState);
    }

    private void beginTimedShootAtScorePose(int nextStateAfterShot) {
        shootSequencer.stopIntake();
        shootSequencer.startShootingSequence(SHOOT_DURATION_MS, SHOOT_TARGET_VELOCITY, SHOOT_VELOCITY_THRESHOLD);
        shootSequenceEndMs = System.currentTimeMillis() + SHOOT_SEQUENCE_WAIT_MS;
        setPathState(nextStateAfterShot);
    }

    private boolean shootWindowElapsed() {
        return System.currentTimeMillis() >= shootSequenceEndMs;
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
        case 0:
            follower.followPath(scorePreload);
            setPathState(1);
            break;
        case 1:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(2);
            }
            break;
        case 2:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(3);
            }
            break;
        case 3:
            if(pathDelayElapsed()) {
                followPathAfterDelay(grabPickup1, 4);
            }
            break;
        case 4:
            if(!follower.isBusy()) {
                setPathState(5);
            }
            break;
        case 5:
            if(pathDelayElapsed()) {
                shootSequencer.stopIntake();
                followPathAfterDelay(scorePickup1, 6);
            }
            break;
        case 6:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(7);
            }
            break;
        case 7:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(8);
            }
            break;
        case 8:
            if(pathDelayElapsed()) {
                followPathAfterDelay(grabPickup2, 9);
            }
            break;
        case 9:
            if(!follower.isBusy()) {
                setPathState(10);
            }
            break;
        case 10:
            if(pathDelayElapsed()) {
                shootSequencer.stopIntake();
                followPathAfterDelay(scorePickup2, 11);
            }
            break;
        case 11:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(12);
            }
            break;
        case 12:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(13);
            }
            break;
        case 13:
            if(pathDelayElapsed()) {
                followPathAfterDelay(grabPickup3, 14);
            }
            break;
        case 14:
            if(!follower.isBusy()) {
                setPathState(15);
            }
            break;
        case 15:
            if(pathDelayElapsed()) {
                shootSequencer.stopIntake();
                followPathAfterDelay(scorePickup3, 16);
            }
            break;
        case 16:
            if(!follower.isBusy()) {
                beginTimedShootAtScorePose(17);
            }
            break;
        case 17:
            if (shootWindowElapsed()) {
                shootSequencer.startIntake();
                setPathState(18);
            }
            break;
        case 18:
            if(pathDelayElapsed()) {
                followPathAfterDelay(park, 19);
            }
            break;
        case 19:
            if(!follower.isBusy()) {
                shootSequencer.stopIntake();
                shootSequencer.setShooterRunningAfterShoot(false);
                shootSequencer.stopShootingSequence();
                setPathState(-1);
            }
            break;
    }
}

/** These change the states of the paths and actions. It will also reset the timers of the individual switches **/
public void setPathState(int pState) {
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
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        shootSequencer.loop(false);

        // Feedback to Driver Hub for debugging
        telemetry.addData("path state", pathState);
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