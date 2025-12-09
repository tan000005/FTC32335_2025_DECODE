package org.firstinspires.ftc.teamcode;


import android.os.UserManager;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

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
