package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.ControllerParams;

public class HoodController {
    private String servoName       = ControllerParams.HW_HOOD_SERVO;
    private static final double MIN_DEGREES = 0.0;
    private static final double MAX_DEGREES = 180.0;

    private Servo hoodServo;
    private double hoodMinPosition = ControllerParams.HOOD_MIN_POSITION;
    private double hoodMaxPosition = ControllerParams.HOOD_MAX_POSITION;

    public void init(HardwareMap hardwareMap ) {
        hoodServo = hardwareMap.get(Servo.class, servoName);
        hoodServo.setPosition(hoodMinPosition);
    }

    // public void setAngle(double degrees) {
    //     double clippedDegrees = Math.max(MIN_DEGREES, Math.min(MAX_DEGREES, degrees));
    //     double normalizedDegrees = (clippedDegrees - MIN_DEGREES) / (MAX_DEGREES - MIN_DEGREES);
    //     double servoPosition = hoodMinPosition + normalizedDegrees * (hoodMaxPosition - hoodMinPosition);
    //     hoodServo.setPosition(servoPosition);
    // }

    public void setPosition(double position) {
        double clippedPosition = Math.max(hoodMinPosition, Math.min(hoodMaxPosition, position));
        hoodServo.setPosition(clippedPosition);
    }

    public double getPosition() {
        return hoodServo.getPosition();
    }

    public double getAngle() {
        double normalizedPosition = (hoodServo.getPosition() - hoodMinPosition) / (hoodMaxPosition - hoodMinPosition);
        return normalizedPosition * (MAX_DEGREES - MIN_DEGREES) + MIN_DEGREES;
    }
}