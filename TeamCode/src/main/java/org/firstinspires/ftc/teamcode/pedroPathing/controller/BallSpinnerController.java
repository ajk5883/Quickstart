package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class BallSpinnerController {
    private DcMotorSimple spinnerMotor;

    public void init(HardwareMap hardwareMap, String motorName) {
        spinnerMotor = hardwareMap.get(DcMotorSimple.class, motorName);
    }

    public void turnOn() {
        spinnerMotor.setPower(1.0);
    }

    public void turnOff() {
        spinnerMotor.setPower(0.0);
    }
}