package org.firstinspires.ftc.teamcode.OpModes_Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotConfiguration;

@Disabled
@Autonomous(name="Display Name") //, preselectTeleOp = "TeleOpMode", group="Name of Group")
public class AutoOp_Template extends RobotConfiguration {


    @Override
    public void runOpMode() throws InterruptedException {

        initializeRobot();

        waitForStart();

//        while (opModeIsActive()) {
//        }
    }

}
