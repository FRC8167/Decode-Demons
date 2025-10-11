package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

public class Shooter implements TeamConstants {
    private DcMotorEx shooterMotor;
    private double  pgain;
    private double shooterSpeed;
    private final double maxSpeed = 28000;


    public Shooter(DcMotorEx motor) {
        shooterMotor = motor;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setPower(0);

        shooterSpeed = 0;
        pgain = 0;
        motor.setVelocityPIDFCoefficients(pgain,0,0,0);
    }


    public void setMotorPower(double rangeInM) {
        shooterSpeed = rangeInM*1;
        shooterMotor.setVelocity(Range.clip(shooterSpeed,-1,1)); //0,maxSpeed));
    }


    public void increaseShooterMotorSpeed(){
        shooterSpeed += 0.2;
        //setMotorSpeed(shooterSpeed);
        setMotorPower(shooterSpeed);
    }


    public void decreaseShooterMotorSpeed(){
        shooterSpeed -= 0.2;
       // setMotorSpeed(shooterSpeed);
        setMotorPower(shooterSpeed);
    }


    public void setMotorSpeed(double speed){
        shooterMotor.setVelocity(speed); //Range.clip(speed,0,maxSpeed), AngleUnit.DEGREES);
    }


    public double getShooterSpeed() {
        return shooterMotor.getVelocity(AngleUnit.DEGREES);
    }


}
