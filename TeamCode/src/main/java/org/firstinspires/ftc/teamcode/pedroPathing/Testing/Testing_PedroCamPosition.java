package org.firstinspires.ftc.teamcode.pedroPathing.Teleop;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.CameraController;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Testing_PedroCamPosition extends OpMode {
    private Follower follower;
    private CameraController cameraController;
    public static Pose startingPose =new Pose(72, 72, 0); //See ExampleAuto to understand how to use this
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private boolean slowMode = false;
    private double slowModeMultiplier = 0.5;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        cameraController = new CameraController();
        cameraController.init(hardwareMap, CameraController.RED_GOAL_PIPELINE);
        follower.setStartingPose(startingPose == null ? new Pose(72, 72, 0) : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(72+24+24, 72+24+24,0))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void start() {
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();
    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (!slowMode) follower.setTeleOpDrive(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x,
                    -gamepad1.right_stick_x,
                    true // Robot Centric
            );

            //This is how it looks with slowMode on
            else follower.setTeleOpDrive(
                    -gamepad1.left_stick_y * slowModeMultiplier,
                    -gamepad1.left_stick_x * slowModeMultiplier,
                    -gamepad1.right_stick_x * slowModeMultiplier,
                    true // Robot Centric
            );
        }

        //Automated PathFollowing
        if (gamepad1.aWasPressed()) {
            follower.followPath(pathChain.get());
            automatedDrive = true;
        }

        //Stop automated following if the follower is done
        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
            follower.startTeleopDrive();
            automatedDrive = false;
        }

        //Slow Mode
        if (gamepad1.rightBumperWasPressed()) {
            slowMode = !slowMode;
        }

        //Optional way to change slow mode strength
        if (gamepad1.xWasPressed()) {
            slowModeMultiplier += 0.25;
        }

        //Optional way to change slow mode strength
        if (gamepad2.yWasPressed()) {
            slowModeMultiplier -= 0.25;
        }

        Pose pedroPose = follower.getPose();
        Pose cameraPose = cameraController.getRobotPose();
        Pose poseDifference = null;
        if (cameraPose != null) {
            poseDifference = new Pose(
                    pedroPose.getX() - cameraPose.getX(),
                    pedroPose.getY() - cameraPose.getY(),
                    normalizeRadians(pedroPose.getHeading() - cameraPose.getHeading())
            );
        }

        telemetryM.debug("position", pedroPose);
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
        telemetryM.debug("cameraTagCount", cameraController.getTagCount());
        telemetryM.debug("cameraPose", cameraPose);
        telemetryM.debug("poseDifference", poseDifference);

        telemetry.addData("position", pedroPose);
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.addData("cameraTagCount", cameraController.getTagCount());
        telemetry.addData("cameraPose", cameraPose);
        telemetry.addData("poseDifference", poseDifference);
        telemetry.update();
    }

    private double normalizeRadians(double angleRadians) {
        return Math.atan2(Math.sin(angleRadians), Math.cos(angleRadians));
    }
}