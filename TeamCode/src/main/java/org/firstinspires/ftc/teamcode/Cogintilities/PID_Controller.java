package org.firstinspires.ftc.teamcode.Cogintilities;

public class PID_Controller {

    private double kP, kI, kD, kF, setPoint;
    private double integralSum = 0;
    private double lastError = 0;
    private long lastTime;

    private double integralLimit = 1.0;

    public PID_Controller(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
        lastTime = System.currentTimeMillis();
    }


    public double command(double feedback) {
        double error;
        long dt;

        error = setPoint - feedback;
        dt = System.currentTimeMillis() - lastTime;

        integralSum += error * dt;
        integralSum = Math.max(-integralLimit, Math.min(integralLimit, integralSum));

        double derivative = (error - lastError) / dt;
        lastTime = System.currentTimeMillis();
        lastError = error;

        return (kP * error) + (kI * integralSum) + (kD * derivative) + kF;
    }


    public void reset() {
        integralSum = 0;
        lastError = 0;
        lastTime = System.currentTimeMillis();
    }


    public void setPoint(double targetValue) {
        setPoint = targetValue;
        reset();
    }


    public void setIntegralLimit(double limit) {
        this.integralLimit = limit;
    }


}
