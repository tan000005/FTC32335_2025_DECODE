package org.firstinspires.ftc.teamcode.RobotClasses.Turret;

import android.telecom.Conference;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.RobotClasses.ModuleControllers.DefaultServoController;

public class AngleAdjustServo {

    public static double angle;
    private DefaultServoController servo;
    private int angleAdjust = Config.angleAdjust;

    public void init(HardwareMap hardwareMap) {
        Servo servoName = hardwareMap.get(Servo.class, Config.angleAdjustServoName); // get the servo name and pass the servo name that we want from the config file
        servo = new DefaultServoController(servoName);

        servo.setServoPosition(0); // set servo position to zero
    }

    public void updatePosition(double range, Telemetry telemetry) {

        if (range < 0)                                                               // if no tag is found (range = -1) then set it to default position
            servo.setServoPosition(Config.defaultShooterServoAngle+angleAdjust); // set to default pos (45 deg) and then add the angle adjustment to it (20 deg)
        else {

            telemetry.addLine("Aiming");

            telemetry.addData("angle: ", angle);

            angle = 120-((range-40)*2);

            if (range < 38) {
                servo.setServoPosition(0);
            } else {
                servo.setServoPosition((int)(angle));
                //servo.setServoPosition((int)(angle)*2);
            }

        }

    }

}
