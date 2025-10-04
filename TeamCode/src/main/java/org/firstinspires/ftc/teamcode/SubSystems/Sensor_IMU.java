package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class Sensor_IMU {

    /** Class Variable Go Here **/
    IMU imu;

    double yawAngle, yawRate;

    // Static variable to hold a single_instance of type Singleton
    private static Sensor_IMU single_instance = null;

    // Constructor
    // Here we will be creating private constructor restricted to this class itself
    private Sensor_IMU(IMU imu_sensor) {
        imu = imu_sensor;

        // This needs to be changed to match the orientation on your robot
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =   RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }

    // Static method to create instance of Singleton class
    // Add any arguments required to create the object
    public static synchronized Sensor_IMU getInstance(IMU imu_sensor)
    {
        if (single_instance == null)
            single_instance = new Sensor_IMU(imu_sensor);

        return single_instance;
    }



    /* ********************* Class Member Functions ********************* */


    public void readIMU() {
        double[] imuData;
        yawAngle = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        yawRate = imu.getRobotAngularVelocity(AngleUnit.RADIANS).zRotationRate;
    }


    public double currentHeading() { return yawAngle; }
    public double currentHeadingDeg() { return yawAngle * 180 / Math.PI; }
    public double yawRate() { return yawRate; }
    public double yawRateDPS() { return yawRate; }

    public void zeroize() {
        imu.resetYaw();
    }

}
