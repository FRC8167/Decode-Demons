package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

public class Shooter implements TeamConstants {
    private DcMotorEx shooterMotor;
    private double  pgain;
    private double shooterSpeed;

    private final int ticksPerRev = 28;
    private final double maxSpeed = 28000;


    public Shooter(DcMotorEx motor) {
        shooterMotor = motor;
        motor.setDirection(DcMotorSimple.Direction.FORWARD);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
//        motor.setPower(0);
        motor.setVelocity(0);

        shooterSpeed = 0;
        pgain = 1;
        motor.setVelocityPIDFCoefficients(pgain,0,0,0);
    }


    public void shootFromDistance(double rangeInM) {
        shooterSpeed = rangeToSpeedTransferFunction(rangeInM);
//        shooterMotor.setVelocity(Range.clip(shooterSpeed,0, maxSpeed));
        setMotorSpeed(shooterSpeed);
    }


    public void increaseShooterMotorSpeed(){
        shooterSpeed += 200;
        setMotorSpeed(shooterSpeed);
    }


    public void decreaseShooterMotorSpeed(){
        shooterSpeed -= 200;
        setMotorSpeed(shooterSpeed);
    }


    /**
     * Set the shooter motor speed
     * @param rpm Desired speed of the motor in RPM
     */
    public void setMotorSpeed(double rpm){
        shooterMotor.setVelocity(rpm / 60 * ticksPerRev); //Range.clip(speed,0,maxSpeed));
    }


    /**
     * Transfer function to set motor speed based on the range to the target.
     * @param range Distance to the target in meters.
     * @return shootor motor rpm
     */
    private double rangeToSpeedTransferFunction(double range) {
        return range * 1 + 0;
    }


    /**
     * Return the velocity of the shooter motor in RPM
     * @return Shooter motor RPM
     */
    public double getShooterSpeed() {
        return shooterMotor.getVelocity() / ticksPerRev * 60;
    }


}
