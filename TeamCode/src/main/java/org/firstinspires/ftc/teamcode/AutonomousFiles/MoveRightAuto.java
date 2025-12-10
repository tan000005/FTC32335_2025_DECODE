package org.firstinspires.ftc.teamcode.AutonomousFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.RobotController;

@Autonomous(name = "MoveRightAutonomous")
public class MoveRightAuto extends LinearOpMode {

    RobotController robotController;

    @Override
    public void runOpMode() {

        robotController = new RobotController();
        robotController.init(hardwareMap, gamepad1);

        waitForStart();

        robotController.strafeRight(100);

    }

}
