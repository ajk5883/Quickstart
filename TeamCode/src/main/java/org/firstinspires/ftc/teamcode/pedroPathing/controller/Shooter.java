package org.firstinspires.ftc.teamcode.pedroPathing;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Shooter {
    private DcMotorEx shooterMotor1;
    private DcMotorEx shooterMotor2;
    private double targetVelocity;
    private double velocityThreshold;

    public void init(HardwareMap hardwareMap, String motor1Name, String motor2Name) {
        shooterMotor1 = hardwareMap.get(DcMotorEx.class, motor1Name);
        shooterMotor2 = hardwareMap.get(DcMotorEx.class, motor2Name);
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public void setVelocityThreshold(double velocityThreshold) {
        this.velocityThreshold = velocityThreshold;
    }

    public void startShooter(double targetVelocity) {
        setTargetVelocity(targetVelocity);
        shooterMotor1.setVelocity(targetVelocity);
        shooterMotor2.setVelocity(targetVelocity);
    }

    public void stopShooter() {
        shooterMotor1.setPower(0);
        shooterMotor2.setPower(0);
    }

    public boolean isVelocityWithinThreshold() {
        double motor1Error = Math.abs(shooterMotor1.getVelocity() - targetVelocity);
        double motor2Error = Math.abs(shooterMotor2.getVelocity() - targetVelocity);
        return motor1Error <= velocityThreshold && motor2Error <= velocityThreshold;
    }
}
