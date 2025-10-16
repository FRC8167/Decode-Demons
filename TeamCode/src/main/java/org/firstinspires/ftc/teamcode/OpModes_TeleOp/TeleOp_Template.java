package org.firstinspires.ftc.teamcode.OpModes_TeleOp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.RobotConfiguration;

@Disabled
@TeleOp(name="TeleOpTemplate", group="Competition")
public class TeleOp_Template extends RobotConfiguration {

    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
