package org.firstinspires.ftc.teamcode.SubSystems;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration;

//@Disabled
@TeleOp(name="IntakeShooterTest")
public class IntakeShooterTeleOp extends RobotConfiguration {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();
        waitForStart();

        while (opModeIsActive()) {

            if (gamepad1.right_bumper) {
                intake.IntakeOn();
            }

            if (gamepad1.left_bumper) {
                intake.IntakeOff();
            }

            if (gamepad1.aWasPressed()) {
                shooter.increaseShooterMotorSpeed();
            }

            if (gamepad1.bWasPressed()) {
                shooter.decreaseShooterMotorSpeed();
            }

            telemetry.addData("Shooter Velocity = ", shooter.getShooterSpeed());

            telemetry.update();
        }
    }
}
