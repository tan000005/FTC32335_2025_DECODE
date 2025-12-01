package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.RobotClasses.Others.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.RobotController;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret.TurretController;

@Autonomous(name = "autonomous", group = "Test")
public class auto extends LinearOpMode {

    RobotController robotController;
    TurretController turretController;
    Intake intake;

    @Override
    public void runOpMode() {

        robotController = new RobotController();
        robotController.autoInit(hardwareMap);

        turretController = new TurretController(hardwareMap);

        intake = new Intake(hardwareMap);

        waitForStart();

        telemetry.addLine("Program started");
        telemetry.update();

        intake.start();

        robotController.moveForward(250);
        sleep(200);

        // shoot preloaded shots using turret

        turretController.turnTurretAround(0);
        turretController.autoShoot(0.5);
        sleep(200);
        for (int i = 0; i < 3; i++) {
            turretController.kick();
            sleep(100);
            turretController.unKick();
            sleep(100);
        }
        sleep(200);
        turretController.turnTurretAround(0);
        sleep(200);

        robotController.turnTo(135);
        turretController.turnTurret45(0);
        sleep(200);
        robotController.strafeLeft(25);
        sleep(200);
        //start intake
        robotController.moveForward(100);
        sleep(200);
        robotController.moveBackward(100);
        sleep(200);

        //shoot

        for (int i = 0; i < 3; i++) {
            turretController.kick();
            sleep(100);
            turretController.unKick();
            sleep(100);
        }

        sleep(200);
        robotController.turnTo(-45);
        sleep(200);
        robotController.strafeLeft(50);
        sleep(200);
        robotController.moveForward(100);
        sleep(200);
        robotController.moveBackward(100);
        sleep(200);
        robotController.strafeRight(50);
        sleep(200);
        robotController.turnTo(45);
        sleep(200);

        //shoot

        for (int i = 0; i < 3; i++) {
            turretController.kick();
            sleep(100);
            turretController.unKick();
            sleep(100);
        }

        sleep(200);
        robotController.turnTo(-45);
        sleep(200);
        robotController.strafeLeft(100);
        sleep(200);
        robotController.moveForward(100);
        sleep(200);
        robotController.moveBackward(100);
        sleep(200);
        robotController.strafeRight(100);
        sleep(200);
        //shoot

        for (int i = 0; i < 3; i++) {
            turretController.kick();
            sleep(100);
            turretController.unKick();
            sleep(100);
        }

        telemetry.addLine("Done");
        telemetry.update();

    }

}
