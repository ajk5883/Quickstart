package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;



public class BallSpinnerController {
    private DcMotorSimple spinnerMotor;
    private String motorName = "bslr";

    public void init(HardwareMap hardwareMap) {
        spinnerMotor = hardwareMap.get(DcMotorSimple.class, motorName);
        spinnerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turnOn() {
        spinnerMotor.setPower(-1.0);
    }

    public void turnOff() {
        spinnerMotor.setPower(0.0);
    }
}