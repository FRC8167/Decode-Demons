package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Cogintilities.PID_Controller;
import org.firstinspires.ftc.teamcode.Cogintilities.TeamConstants;

public class Shooter_CustomPID implements TeamConstants {
    private final  DcMotorEx shooterMotor;
    private double kP;
    private double shooterSpeed;

    private final int    ticksPerRev = 28;
    private final double maxSpeed = 6000;

    PID_Controller pid;


    public Shooter_CustomPID(DcMotorEx motor) {
        shooterMotor = motor;

        shooterMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        shooterMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        shooterSpeed = 0;
        shooterMotor.setPower(0);

        kP = 2;
        pid = new PID_Controller(kP,0,0,0);

    }


    public void shootFromDistance(double rangeInM) {
        shooterSpeed = rangeToSpeedTransferFunction(rangeInM);
        setTarget(shooterSpeed);
    }


    public void increaseShooterMotorSpeed(){
        shooterSpeed += 500;
        setTarget(shooterSpeed);
    }


    public void decreaseShooterMotorSpeed(){
        shooterSpeed -= 500;
        setTarget(shooterSpeed);
    }


    /**
     * Transfer function to set motor speed based on the range to the target.
     * @param range Distance to the target in meters.
     * @return shootor motor rpm
     */
    private double rangeToSpeedTransferFunction(double range) {
        return range * 1 + 0;
    }





    /* ************************ Alternate PID Control Using Motor Power ************************  */

    /* Steps to implement custom PID in teleOp
    *  1. Call setTarget with the desired RPM to maintain
    *  2. The update() function needs to be called once per runOpMode loop
    */


    /**
     * Set the power of the motor
     * @param pwr Power level to set the the motor to
     */
    private void setMotorPower(double pwr) {
        shooterMotor.setPower(Range.clip(pwr, -1, 1));
    }


    /**
     * Change the PID controller setpoint that the controller is trying to maintain
     * @param rpm  New target velocity to maintain
     */
    public void setTarget(double rpm) {
        pid.setPoint(rpm);
    }


    /**
     * Use this function when the built in motor encoder is being used for velocity control
     * feedback.  It must be called once per runOpMode loop
     */
    public void updateVelocityCtrl() {
        setMotorPower(pid.command(shooterMotor.getVelocity()));
    }


    /**
     * Use this function when a feedback sensor other than the built in encoder is being used.
     * @param feedbackSensorValue Value of the sensor being used for PID control
     */
    public void update(double feedbackSensorValue) {
        setMotorPower(pid.command(feedbackSensorValue));
    }


    /**
     * Return the shooterMotors currently configured power level.
     * @return Current power level of the motor. [-1 to 1]
     */
    public double getShooterMotorPwr() {
        return shooterMotor.getPower();
    }


    /**
     * Return the velocity of the shooter motor in RPM
     * @return Shooter motor RPM
     */
    public double getShooterSpeed() {
        return shooterMotor.getVelocity() / ticksPerRev * 60;
    }

    public double getShooterVar() {
        return shooterSpeed;
    }
    public int getEncoderCnts() {
        return shooterMotor.getCurrentPosition();
    }



}
