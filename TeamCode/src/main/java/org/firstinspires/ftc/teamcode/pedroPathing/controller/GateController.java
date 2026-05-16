package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.ControllerParams;

public class GateController {
    private Servo gateServo;
    private double openPosition   = ControllerParams.GATE_OPEN_POSITION;
    private double closedPosition  = ControllerParams.GATE_CLOSED_POSITION;
    private String servoName       = ControllerParams.HW_GATE_SERVO;

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