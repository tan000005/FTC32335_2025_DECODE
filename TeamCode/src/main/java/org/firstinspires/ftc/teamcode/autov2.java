package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotClasses.Others.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.RobotController;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret.TurretController;
import org.firstinspires.ftc.teamcode.RobotClasses.VisionProcessing.VisionProcessing;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name = "autov2", group = "final")
public class autov2 extends LinearOpMode {

    RobotController robotController;
    TurretController turret;
    Intake intake;

    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private WebcamName webcamName;
    private VisionProcessing visionProcessor;

    @Override
    public void runOpMode() {

        // builds webcam name
        webcamName = hardwareMap.get(WebcamName.class, Config.webcamName);

        // builds vision processing class
        visionProcessor = new VisionProcessing();
        visionProcessor.init(hardwareMap, telemetry);

        robotController = new RobotController();
        robotController.autoInit(hardwareMap);

        turret = new TurretController(hardwareMap);

        intake = new Intake(hardwareMap);

        Runnable trackingTask = new Runnable() {
            @Override
            public void run() {
                while (opModeIsActive()) {
                    visionProcessor.update();
                    turret.autoUpdate(visionProcessor.getTagDataBySpecificId(23),telemetry);
                }
            }
        };

        Thread trackingThread = new Thread(trackingTask);
        trackingThread.start();

        telemetry.addLine("Tracking thread started");
        telemetry.update();

        waitForStart();

        intake.start();

        // move forward to postion
        robotController.moveForward(120);
        sleep(100);
        // turn to face the balls headon
        robotController.turnTo(135);
        sleep(100);
        // strafe into position
        robotController.strafeLeft(50);
        sleep(100);
        // move forwards to collect the balls
        robotController.moveForward(100);
        sleep(100);
        // move backwards into shooting position
        robotController.moveBackward(80);
        // launch the balls
        intake.launch();
        sleep(1000);
        intake.start(); // reset intake to normal speed
        // strafe into position for the next ball
        robotController.strafeLeft(50);
        sleep(100);
        // move forward to collect balls
        robotController.moveForward(80);
        sleep(100);
        // move backwards into shooting position
        robotController.moveBackward(80);
        sleep(100);
        // shoot
        intake.launch();
        sleep(1000);
        intake.start();
        //strafe into next position
        robotController.strafeLeft(50);
        sleep(100);
        // move foward to collect balls
        robotController.moveForward(80);
        sleep(100);
        // move backwards into shooting position
        robotController.moveBackward(100);
        sleep(100);
        // shoot
        intake.launch();
        sleep(1000);
        intake.start();

    }

}
