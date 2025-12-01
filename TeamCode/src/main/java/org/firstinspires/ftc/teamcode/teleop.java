package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.RobotClasses.ModuleControllers.MG996RController;
import org.firstinspires.ftc.teamcode.RobotClasses.Others.Intake;
import org.firstinspires.ftc.teamcode.RobotClasses.Turret.TurretController;
import org.firstinspires.ftc.teamcode.RobotClasses.VisionProcessing.TagLibrary;
import org.firstinspires.ftc.teamcode.RobotClasses.VisionProcessing.VisionProcessing;
import org.firstinspires.ftc.teamcode.RobotClasses.RobotController;

import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.VisionProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import org.opencv.core.Mat;

@TeleOp(name="Teleop")
public class teleop extends OpMode {

    // vision processing related variables
    private AprilTagProcessor aprilTagProcessor;
    private VisionPortal visionPortal;
    private WebcamName webcamName;
    private VisionProcessing visionProcessor;
    private RobotController robotController;

    TurretController turret;
    Intake intake;

    @Override
    public void init() {

        // builds webcam name
        webcamName = hardwareMap.get(WebcamName.class, Config.webcamName);

        // builds the vision processing class
        visionProcessor = new VisionProcessing();
        visionProcessor.init(hardwareMap, telemetry);

        // intake init
        intake = new Intake(hardwareMap);

        // turret init
        turret = new TurretController(hardwareMap);

        robotController = new RobotController();
        robotController.init(hardwareMap, gamepad1);


    }

    @Override
    public void loop() {

        visionProcessor.update();
        turret.update(visionProcessor.getTagDataBySpecificId(23), telemetry, gamepad1);
        robotController.updatePosition();
        intake.updateIntake(gamepad1);

    }

    @Override
    public void stop() {

        telemetry.addLine("Program ended");
        telemetry.update();

    }

}
