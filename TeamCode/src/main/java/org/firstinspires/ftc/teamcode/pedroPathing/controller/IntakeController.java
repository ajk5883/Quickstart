package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class IntakeController {
    private DcMotorSimple intakeMotor;
    private String motorName = "intakefb";

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorSimple.class, motorName);
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void turnOnIntake() {
        intakeMotor.setPower(1.0);
    }

    public void turnOffIntake() {
        intakeMotor.setPower(0.0);
    }
}
