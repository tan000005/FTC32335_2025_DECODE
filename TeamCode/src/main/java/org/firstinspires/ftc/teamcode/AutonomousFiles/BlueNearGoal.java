package org.firstinspires.ftc.teamcode.AutonomousFiles;

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

    public void shoot() {

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

    }

    @Override
    public void runOpMode() {

        robotController = new RobotController();
        robotController.autoInit(hardwareMap);

        turretController = new TurretController(hardwareMap);

        intake = new Intake(hardwareMap);

        telemetry.addLine("Robot ready to start");

        waitForStart();

        turretController.autoShoot(-0.85);

        // backwards
        robotController.moveForward(100);
        //sleep(100);
        robotController.turnTo(180);
        shoot();
        robotController.turnTo(-225);
        robotController.moveForward(100);
    }

}
