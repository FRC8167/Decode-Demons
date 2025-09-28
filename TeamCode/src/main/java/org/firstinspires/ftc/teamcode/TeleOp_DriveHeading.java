package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

//@Disabled
@TeleOp(name="Drive with Heading")
public class TeleOp_DriveHeading extends RobotConfiguration {

    double headingToMaintain;

    @Override
    public void runOpMode() throws InterruptedException {

        Gamepad driver = gamepad1;

        initializeRobot();
        headingToMaintain = 90.0;

        waitForStart();

        while (opModeIsActive()) {

            /* IMU reads are expensive. Read once and pull data from class variables */
            imu.readIMU();

            /* Left Bumper enables drive with a constant heading, right bumper is field centric drive and
            no buttons is normal drive */
            if(driver.left_bumper) {
                drive.driveWithHeading(driver.left_stick_y, driver.left_stick_x, driver.right_stick_x, imu.currentHeading(), headingToMaintain);
                telemetry.addData("Mode", "Constant Heading");
                telemetry.addData("Yaw (Z)", "%.2f Deg. (Heading)", imu.currentHeadingDeg());
                telemetry.addData("Yaw Error", "%.2f Deg", imu.currentHeadingDeg() - headingToMaintain);
            } else if(driver.right_bumper){
                drive.driveFieldRelative(driver.left_stick_y, driver.left_stick_x, driver.right_stick_x, imu.currentHeading());
                telemetry.addData("Mode", "Field Centric");
            } else {
                drive.mecanumDrive(driver.left_stick_y, driver.left_stick_x, driver.right_stick_x);
                telemetry.addData("Mode", "Normal Drive");
            }

            telemetry.update();
        }
    }
}
