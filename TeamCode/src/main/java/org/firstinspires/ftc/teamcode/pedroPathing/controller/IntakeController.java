package org.firstinspires.ftc.teamcode.pedroPathing.controller;

import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.pedroPathing.controller.ControllerParams;

public class IntakeController {
    private DcMotorSimple intakeMotor;
    private String motorName = ControllerParams.HW_INTAKE_MOTOR;

    public void init(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorSimple.class, motorName);
        intakeMotor.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    public void turnOnIntake() {
        intakeMotor.setPower(ControllerParams.INTAKE_POWER_FORWARD);
    }

    public void turnOnIntakeReverse() {
        intakeMotor.setPower(ControllerParams.INTAKE_POWER_REVERSE);
    }

    public void turnOffIntake() {
        intakeMotor.setPower(0.0);
    }
}
