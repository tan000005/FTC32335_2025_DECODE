package org.firstinspires.ftc.teamcode.RobotClasses.Turret;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Config;

import org.firstinspires.ftc.teamcode.RobotClasses.VisionProcessing.VisionProcessing;
import org.firstinspires.ftc.vision.VisionPortal;


public class TurretController {

    DcMotor MOTOR;
    DcMotor SHOOTING_MOTOR;

    int count = 0;

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

    public void autoShoot(double speed) { SHOOTING_MOTOR.setPower(speed); }

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
                shootingMotorSpeed = -1.0;
                state = true;
            } else {
                shootingMotorSpeed = 0.0;
                state = false;
            }
        }

    }

    public void update(double[] xyhv, Telemetry telemetry, Gamepad gamepad) {

        double range;

        if (xyhv == null) range = 100;
        else range = xyhv[4];

        updateControls(gamepad, range);

        if (xyhv != null) {

            timer.reset();

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

            if (timer.seconds() > 1) {

                telemetry.addLine("No tag detected");
                int currentMotorPosition = MOTOR.getCurrentPosition();

                if (currentMotorPosition >= 1400 || currentMotorPosition <= -1400) {
                    count++;
                }

                if (currentMotorPosition <= 1400 && count % 2 == 0) {
                    MOTOR.setPower(0.5);
                }
                else if (currentMotorPosition >= -1400 && count % 2 != 0) {
                    MOTOR.setPower(-0.5);
                } else {
                    MOTOR.setPower(0);
                }

                telemetry.addData("encoder position: ", currentMotorPosition);
                telemetry.addData("motor power: ", MOTOR.getPower());

            } else {
                telemetry.addLine("Waiting for delzy to be over");
            }
        }
    }

    public void autoUpdate(double[] xyhv, Telemetry telemetry) {

        if (xyhv != null) {

            timer.reset();

            telemetry.addData("x    :   ", xyhv[0]);
            telemetry.addData("y    :   ", xyhv[1]);
            telemetry.addData("r    :   ", xyhv[4]);

            angleAdjustServo.updatePosition(xyhv[4]);

            MOTOR.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            if (xyhv[0] < -Config.TURRET_DEADZONE) {
                telemetry.addLine("1");
                MOTOR.setPower(-calculateSpeed(xyhv));
            } else {
                telemetry.addLine("-1");
                MOTOR.setPower(-calculateSpeed(xyhv));
            }
        }
        else {

            if (timer.seconds() > 1) {

                telemetry.addLine("No tag detected");
                int currentMotorPosition = MOTOR.getCurrentPosition();

                if (currentMotorPosition >= Config.TURRET_IDLE_TICKS) {
                    MOTOR.setPower(0.5);
                } else {
                    MOTOR.setPower(-0.5);
                }

                telemetry.addData("encoder position: ", currentMotorPosition);
                telemetry.addData("motor power: ", MOTOR.getPower());

            } else {
                telemetry.addLine("Waiting for delzy to be over");
            }
        }
    }

    public void turnTurretAround(int offset) {
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR.setTargetPosition(440+offset);
        MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR.setPower(0.5);
    }

    public void turnTurret45(int offset) {
        MOTOR.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        MOTOR.setTargetPosition(220+offset);
        MOTOR.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        MOTOR.setPower(0.5);
    }

    public double calculateSpeed(double[] xyhv) {
        double error = xyhv[0];
        double servoPower = Config.KP * error/20;

        servoPower = Math.max(-1.0, Math.min(1.0, servoPower));

        return servoPower;

    }

}
