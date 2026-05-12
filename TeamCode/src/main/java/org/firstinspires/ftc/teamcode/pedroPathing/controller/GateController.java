package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GateController {
    private Servo gateServo;
    private double openPosition=0.0;
    private double closedPosition=0.25;
    private String servoName = "gate";

    public void init(HardwareMap hardwareMap) {
        gateServo = hardwareMap.get(Servo.class, servoName);
        closeGate();
    }

    public void openGate() {
        gateServo.setPosition(openPosition);
    }

    public void closeGate() {
        gateServo.setPosition(closedPosition);
    }

    public void setPosition(double position) {
        gateServo.setPosition(Math.max(0.0, Math.min(1.0, position)));
    }

    public double getPosition() {
        return gateServo.getPosition();
    }
}