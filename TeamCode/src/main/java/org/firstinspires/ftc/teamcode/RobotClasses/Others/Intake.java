package org.firstinspires.ftc.teamcode.RobotClasses.Others;

import static android.os.SystemClock.sleep;

import android.net.wifi.aware.IdentityChangedListener;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;

public class Intake {

    DcMotor intakeMotor;
    ElapsedTime elapsedTime;

    boolean canShoot = true;

    public Intake(HardwareMap hardwareMap) {

        this.intakeMotor = hardwareMap.get(DcMotor.class, Config.INTAKE_MOTOR_NAME);
        this.intakeMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        this.elapsedTime = new ElapsedTime();

    }

    public void start() {
        intakeMotor.setPower(1);
    }

    public void update_message(Telemetry telemetry) {
        if (elapsedTime.seconds() > 1) {
            telemetry.addLine("CAN SHOOT");
        } else {
            telemetry.addLine("DON'T SHOOT");
        }
        telemetry.update();
    }

    public void stop() {
        intakeMotor.setPower(0);
    }

    public void reverse() {
        intakeMotor.setPower(-0.7);
    }

    public void autoShoot() {
        intakeMotor.setPower(-1);
        sleep(400);
        intakeMotor.setPower(0);
        sleep(1000);
        intakeMotor.setPower(-1);
        sleep(400);
        intakeMotor.setPower(0);
        sleep(1000);
        intakeMotor.setPower(-1);
        sleep(400);
        intakeMotor.setPower(0);
        sleep(1000);
    }

    public void launch() { intakeMotor.setPower(1); }

    public void updateIntake(Gamepad gamepad) {
        if (gamepad.y) {
            start();
        } else if (gamepad.x || gamepad.left_bumper || gamepad.right_bumper) {
            if (elapsedTime.seconds() > 2) {
                elapsedTime.reset();
            } else {
                return;
            }
            reverse();
        } else {
            stop();
        }
    }

}
