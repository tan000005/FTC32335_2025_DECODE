package org.firstinspires.ftc.teamcode.RobotClasses.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;

public class TurretController {

    DcMotor MOTOR;
    DcMotor SHOOTING_MOTOR;

    AngleAdjustServo angleAdjustServo;
    Servo shootingServo;
    ElapsedTime timer = new ElapsedTime();

    double swingPower = 0.5;
    double shootingMotorSpeed = 0.0;

    boolean state = false;

    public TurretController(HardwareMap hardwareMap) {

        this.MOTOR = hardwareMap.get(DcMotor.class, Config.turretMotorName);
        this.MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        this.MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        this.SHOOTING_MOTOR = hardwareMap.get(DcMotor.class, Config.shootingMotorName);
        this.SHOOTING_MOTOR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        this.SHOOTING_MOTOR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        this.angleAdjustServo = new AngleAdjustServo();

        this.shootingServo = hardwareMap.get(Servo.class, Config.shootingServoName);

        angleAdjustServo.init(hardwareMap);

    }


    public void kick() {
        shootingServo.setPosition(1.0);
        timer.reset();
    }

    public void unKick() {
        shootingServo.setPosition(0.0);
    }

    public void shoot() {
        SHOOTING_MOTOR.setPower(shootingMotorSpeed);
    }

    public double calculateMotorPower(double range) {
        return (range/100);
    }

    public void updateControls(Gamepad gamepad, double range) {

        shoot();

        if (timer.seconds() > 2.0) {
            unKick();
        }

        if (gamepad.x) {
            if (!state) {
                shootingMotorSpeed = 1.0;
                state = true;
            } else {
                shootingMotorSpeed = 0.0;
                state = false;
            }
        }

        if (gamepad.x) {
            kick();
        }

    }

    public void update(double[] xyhv, Telemetry telemetry, Gamepad gamepad) {

        double range;

        if (xyhv == null) range = 100;
        else range = xyhv[4];

        updateControls(gamepad, range);

        if (xyhv != null) {
            telemetry.addData("x    :   ", xyhv[0]);
            telemetry.addData("y    :   ", xyhv[1]);
            telemetry.addData("r    :   ", xyhv[4]);

            angleAdjustServo.updatePosition(xyhv[4]);

            MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (xyhv[0] < -Config.TURRET_DEADZONE) {
                //MOTOR.setPower(1);
                telemetry.addLine("1");
                MOTOR.setPower(-calculateSpeed(xyhv));
            } else {
                telemetry.addLine("-1");
                MOTOR.setPower(-calculateSpeed(xyhv));
            }
        }
        else {
            telemetry.addLine("No tag detected");
            int currentMotorPosition = MOTOR.getCurrentPosition();

            if (currentMotorPosition >= 100) {
                MOTOR.setPower(0.5);
            }
            else {
                MOTOR.setPower(-0.5);
            }

            telemetry.addData("encoder position: ", currentMotorPosition);
            telemetry.addData("motor power: ", MOTOR.getPower());

        }
    }

    /*
    public void update(double[] xyhv, Telemetry telemetry) {

        telemetry.addLine("TESTING SWING ONLY");
        int currentMotorPosition = MOTOR.getCurrentPosition();

        // Check if we hit the "right" limit (100)
        if (currentMotorPosition >= Config.TURRET_IDLE_TICKS) {
            swingPower = -0.3; // Reverse to swing left
        }
        // Check if we hit the "left" limit (-100)
        else if (currentMotorPosition <= -Config.TURRET_IDLE_TICKS) {
            swingPower = 0.3;  // Reverse to swing right
        }

        // Apply the current swing power
        MOTOR.setPower(swingPower);

        telemetry.addData("encoder position: ", currentMotorPosition);
        telemetry.addData("motor power: ", MOTOR.getPower());
    }
    */
    public double calculateSpeed(double[] xyhv) {
        double error = xyhv[0];
        double servoPower = Config.KP * error/20;

        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        return servoPower;

    }

}
