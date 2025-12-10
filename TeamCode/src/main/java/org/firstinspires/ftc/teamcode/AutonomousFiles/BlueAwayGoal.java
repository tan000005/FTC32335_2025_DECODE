package org.firstinspires.ftc.teamcode.AutonomousFiles;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Others.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.RobotController;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret.TurretController;

@Autonomous(name = "BlueAwayGoal", group = "final")
public class BlueAwayGoal extends LinearOpMode {

    RobotController robotController;
    TurretController turretController;
    Intake intake;

    @Override
    public void runOpMode() {

        robotController = new RobotController();
        robotController.autoInit(hardwareMap);

        turretController = new TurretController(hardwareMap);

        intake = new Intake(hardwareMap);

        telemetry.addLine("Robot ready to start");

        waitForStart();

        turretController.autoShoot(-0.83);

        robotController.moveForward(50);
        robotController.turnTo(40);

        intake.reverse();

        turretController.kick();
        sleep(500);
        turretController.unKick();
        sleep(2000);
        turretController.kick();
        sleep(500);
        turretController.unKick();
        sleep(2000);
        turretController.kick();
        sleep(500);
        turretController.unKick();
        sleep(2000);

        /*
        robotController.strafeRight(80);
        robotController.moveForward(125);
        robotController.turnTo(20);

        intake.autoShoot();

        robotController.turnTo(20);

        intake.reverse();

        robotController.moveForward(60);
        robotController.moveBackward(60);

        intake.stop();

        robotController.turnTo(-20);

        intake.autoShoot();

        robotController.turnTo(20);
        robotController.strafeRight(50);

        intake.reverse();

        robotController.moveForward(60);
        robotController.moveBackward(60);
        */
    }

}
