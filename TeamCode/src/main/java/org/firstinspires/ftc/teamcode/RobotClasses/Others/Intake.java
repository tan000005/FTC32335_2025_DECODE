package org.firstinspires.ftc.teamcode.RobotClasses.Others;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Config;

public class Intake {

    DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {

        this.intakeMotor = hardwareMap.get(DcMotor.class, Config.INTAKE_MOTOR_NAME);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void start() {
        intakeMotor.setPower(.6);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void reverse() {
        intakeMotor.setPower(-0.6);
    }

    public void launch() { intakeMotor.setPower(1); }

    public void updateIntake(Gamepad gamepad) {
        if (gamepad.right_bumper) {
            start();
        } else if (gamepad.left_bumper) {
            reverse();
        } else {
            stop();
        }
    }

}
