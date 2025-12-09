package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Others.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.RobotController;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret.TurretController;

@Autonomous(name = "BlueNearGoal", group = "final")
public class BlueNearGoal extends LinearOpMode {

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

        turretController.autoShoot(-0.8);

        // backwards
        robotController.moveForward(100);
        //sleep(100);
        robotController.turnTo(180);
        //sleep(100);
        intake.autoShoot();
        //sleep(100);
        robotController.turnTo(30);
        //sleep(100);
        robotController.strafeLeft(50);
        intake.reverse();
        //sleep(100);
        robotController.moveForward(80);
        //sleep(100);
        robotController.moveBackward(80);
        intake.stop();
        //sleep(100);
        robotController.turnTo(-50);
        //sleep(100);
        intake.autoShoot();
        //sleep(100);
        robotController.turnTo(35);
        //sleep(100);
        robotController.strafeLeft(55);
        intake.reverse();
        robotController.moveForward(80);
        /*
        robotController.moveBackward(50);
        sleep(100);
        // recenter and shoot
        robotController.strafeRight(50);
        sleep(100);
        robotController.moveForward(50);
        sleep(100);
        robotController.moveBackward(50);
        // recenter and shoot
        robotController.strafeRight(50);
        sleep(100);
        robotController.moveForward(50);
        sleep(100);
        robotController.moveBackward(50);
        // recenter and shoot
        */
    }

}
