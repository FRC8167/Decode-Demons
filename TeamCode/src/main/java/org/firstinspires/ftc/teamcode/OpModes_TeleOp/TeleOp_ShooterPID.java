package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration;

//@Disabled
@TeleOp(name="Shooter PID Test")
public class TeleOp_ShooterPID extends RobotConfiguration {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.rightBumperWasPressed()) shooterPID.increaseShooterMotorSpeed();
            if (gamepad1.leftBumperWasPressed())  shooterPID.decreaseShooterMotorSpeed();
            if(gamepad1.xWasPressed()) shooterPID.setTarget(0);

            telemetry.addData("Shooter Speed", "%0.1f rpm", shooterPID.getShooterSpeed());
            telemetry.addData("Shooter Power", "%0.2f", shooterPID.getShooterMotorPwr());
            telemetry.update();

            shooterPID.updateVelocityCtrl();
        }
    }
}
