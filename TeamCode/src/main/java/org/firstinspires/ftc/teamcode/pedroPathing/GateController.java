package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class GateController {
    private Servo gateServo;
    private double openPosition;
    private double closedPosition;

    public void init(HardwareMap hardwareMap, String servoName, double openPosition, double closedPosition) {
        this.openPosition = openPosition;
        this.closedPosition = closedPosition;
        gateServo = hardwareMap.get(Servo.class, servoName);
        closeGate();
    }

    public void openGate() {
        gateServo.setPosition(openPosition);
    }

    public void closeGate() {
        gateServo.setPosition(closedPosition);
    }
}