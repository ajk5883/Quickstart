package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodController {
    private static final double MIN_DEGREES = 0.0;
    private static final double MAX_DEGREES = 180.0;

    private Servo hoodServo;
    private double hoodMinPosition;
    private double hoodMaxPosition;

    public void init(HardwareMap hardwareMap, String servoName, double hoodMinPosition, double hoodMaxPosition) {
        this.hoodMinPosition = hoodMinPosition;
        this.hoodMaxPosition = hoodMaxPosition;
        hoodServo = hardwareMap.get(Servo.class, servoName);
        hoodServo.setPosition(hoodMinPosition);
    }

    public void setAngle(double degrees) {
        double clippedDegrees = Math.max(MIN_DEGREES, Math.min(MAX_DEGREES, degrees));
        double normalizedDegrees = (clippedDegrees - MIN_DEGREES) / (MAX_DEGREES - MIN_DEGREES);
        double servoPosition = hoodMinPosition + normalizedDegrees * (hoodMaxPosition - hoodMinPosition);
        hoodServo.setPosition(servoPosition);
    }
}