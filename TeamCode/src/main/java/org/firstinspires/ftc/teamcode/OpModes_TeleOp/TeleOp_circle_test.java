package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration;

@Disabled
@TeleOp(name="carasoulTest", group="Competition")
public class TeleOp_circle_test extends RobotConfiguration {

    private double circlePower = 0;

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();

        circlePower = 0;
//        circleMotor.setPower(circlePower);


        while (opModeIsActive()) {

            if (gamepad1.left_stick_y > 0) {
//                circleMotor.setPower(gamepad1.left_stick_y);
            }

            if (gamepad1.rightBumperWasPressed()) {
                circlePower += 0.10;
            }

            if (gamepad1.leftBumperWasPressed()) {
                circlePower -= 0.10;
            }

            if (gamepad1.a) {
                circlePower = 0;
            }

//            circleMotor.setPower(circlePower);

            telemetry.addData("power", circlePower);
            telemetry.update();

        }
    }
}