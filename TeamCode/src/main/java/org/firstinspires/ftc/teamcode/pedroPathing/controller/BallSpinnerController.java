package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.ControllerParams;



public class BallSpinnerController {
    private DcMotorSimple spinnerMotor;
    private String motorName = ControllerParams.HW_SPINNER_MOTOR;

    public void init(HardwareMap hardwareMap) {
        spinnerMotor = hardwareMap.get(DcMotorSimple.class, motorName);
        spinnerMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turnOn() {
        spinnerMotor.setPower(ControllerParams.SPINNER_POWER_ON);
    }

    public void turnOff() {
        spinnerMotor.setPower(0.0);
    }
}