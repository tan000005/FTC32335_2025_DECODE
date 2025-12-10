package org.firstinspires.ftc.teamcode.TestClasses;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Config;

@TeleOp(name = "testClass")
public class testClass extends OpMode {

    Servo servo;

    @Override
    public void init() {
        servo = hardwareMap.get(Servo.class, Config.angleAdjustServoName);
    }

    @Override
    public void loop() {
        servo.setPosition(0);
    }

}
