package org.firstinspires.ftc.teamcode.RobotClasses.VisionProcessing;

import android.annotation.SuppressLint;
import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.ArrayList;
import java.util.List;

public class VisionProcessing {
    private AprilTagProcessor aprilTagProcessor;
    public VisionPortal visionPortal;
    private List<AprilTagDetection> detectionTags = new ArrayList<>();

    private Telemetry telemetry;

    public void init(HardwareMap hwMap, Telemetry telemetry){
        this.telemetry = telemetry;

        aprilTagProcessor = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setOutputUnits(DistanceUnit.CM, AngleUnit.DEGREES)
                .setSuppressCalibrationWarnings(true)
                .build();
        VisionPortal.Builder builder = new VisionPortal.Builder();
        builder.setCamera(hwMap.get(WebcamName.class,"Webcam 1"));
        builder.setStreamFormat(VisionPortal.StreamFormat.MJPEG);
        builder.setCameraResolution(new Size(1920,1080));
        builder.addProcessor(aprilTagProcessor);

        visionPortal = builder.build();
    }

    public void update() {
        detectionTags = aprilTagProcessor.getDetections();
    }

    public List<AprilTagDetection> getDetectionTags(){
        return detectionTags;
    }

    @SuppressLint("DefaultLocale")
    public void displayDetectionTelemetry(AprilTagDetection detectionId) {
        if (detectionId == null) {return;}

        if (detectionId.metadata != null) {
            telemetry.addLine(String.format("\n==== (ID %d) %s", detectionId.id, detectionId.metadata.name));
            telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (cm)", detectionId.ftcPose.x, detectionId.ftcPose.y, detectionId.ftcPose.z));
            telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detectionId.ftcPose.pitch, detectionId.ftcPose.roll, detectionId.ftcPose.yaw));
            telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (cm, deg, deg)", detectionId.ftcPose.range, detectionId.ftcPose.bearing, detectionId.ftcPose.elevation));
            telemetry.addLine(String.format("fps: %6.1f", visionPortal.getFps()));
        } else {
            telemetry.addLine(String.format("\n==== (ID %d) Unknown", detectionId.id));
            telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detectionId.center.x, detectionId.center.y));
        }
    }

    public double[] getTagDataBySpecificId(int id)
    {
        for(AprilTagDetection detection : detectionTags){
            if (detection.id == id){
                displayDetectionTelemetry(detection);
                return new double[] {detection.ftcPose.x,
                        detection.ftcPose.y,
                        detection.ftcPose.pitch,
                        detection.ftcPose.yaw,
                        detection.ftcPose.range
                };
            }
        }
        return null;
    }

    public void stop(){
        if (visionPortal != null) {
            visionPortal.close();
        }
    }

}