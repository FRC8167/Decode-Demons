package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

public class Intake implements TeamConstants {
    private DcMotorEx motor;
    private double  pgain;
    private double shooterSpeed = 0.5;

    public Intake(DcMotorEx motor) {
        this.motor = motor;

        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);
    }


    public void setMotorPower(double power) {
//        motor.setPower((power);
    }


    public void IntakeOn() {
        motor.setPower(shooterSpeed);
    }


    public void IntakeOff() {
        motor.setPower(0);
    }



}
