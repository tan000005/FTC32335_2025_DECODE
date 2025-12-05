package org.firstinspires.ftc.teamcode.RobotClasses.Others;

import static android.os.SystemClock.sleep;

import android.net.wifi.aware.IdentityChangedListener;

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
        intakeMotor.setPower(1);
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void reverse() {
        intakeMotor.setPower(-0.6);
    }

    public void autoShoot() {
        intakeMotor.setPower(-1);
        sleep(800);
        intakeMotor.setPower(0);
        sleep(1000);
        intakeMotor.setPower(-1);
        sleep(800);
        intakeMotor.setPower(0);
        sleep(1000);
        intakeMotor.setPower(-1);
        sleep(800);
        intakeMotor.setPower(0);
        sleep(1000);
    }

    public void launch() { intakeMotor.setPower(1); }

    public void updateIntake(Gamepad gamepad) {
        if (gamepad.y) {
            start();
        } else if (gamepad.b) {
            reverse();
        } else {
            stop();
        }
    }

}
