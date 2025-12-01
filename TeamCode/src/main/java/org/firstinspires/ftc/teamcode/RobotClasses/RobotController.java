package org.firstinspires.ftc.teamcode.RobotClasses;

import android.net.MailTo;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.IMU;
import com.sun.tools.javac.util.MandatoryWarningHandler;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Config;
import org.opencv.core.Mat;

public class RobotController {

    private DcMotor FrontLeftMotor;
    private DcMotor FrontRightMotor;
    private DcMotor RearLeftMotor;
    private DcMotor RearRightMotor;

    private Gamepad gamepad;

    private double CurrentFrontLeftPower = 0.0;
    private double CurrentRearLeftPower = 0.0;
    private double CurrentFrontRightPower = 0.0;
    private double CurrentRearRightPower = 0.0;

    private IMU imu;

    private int ticksPerWheelRev;
    private int ticksPerInch;
    private double turnCircumference;
    private double maxSpeed = 0.7;
    private double lastHeading = 0;
    private  double globalAngle = 0;
    private Orientation angles;

    // --- Controller Constants (you will need to tune these) ---
    final double P_GAIN = 0.02;      // How fast to react
    final double HEADING_TOLERANCE = 1.0; // How close is "close enough"
    final double MIN_POWER = 0.15;   // Minimum power to overcome friction
    final double MAX_POWER = 0.8;    // Maximum power to apply

    public void init(HardwareMap hardwareMap, Gamepad gamepd) {

        // set the gamepad to the gamepad that has been passed to us from the Teleop class
        gamepad = gamepd;

        // get and set all the actual hardware for each motor
        FrontLeftMotor = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_DRIVE_NAME);
        RearLeftMotor = hardwareMap.get(DcMotor.class, Config.REAR_LEFT_DRIVE_NAME);
        FrontRightMotor = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_DRIVE_NAME);
        RearRightMotor = hardwareMap.get(DcMotor.class, Config.REAR_RIGHT_DRIVE_NAME);

        // set encoders, can turn off for more direct input but strafing might feel weird
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // reverse one side since mechanum works weirdly
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

    }

    public void updatePosition() {

        // Get input
        double y = -gamepad.right_stick_y; // Forwards / Backwards
        //double x = (gamepad.left_stick_x);
        double x = (gamepad.right_trigger - gamepad.left_trigger) * 1.1;
        double rx = gamepad.left_stick_x; // Rotation

        double theta = Math.atan2(y,x);
        double power = Math.hypot(x,y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        double TargetFrontLeftPower = power * cos/max + rx ;
        double TargetRearLeftPower = power * sin/max + rx;
        double TargetFrontRightPower = power * sin/max - rx;
        double TargetRearRightPower = power * cos/max - rx;

        if ((power + Math.abs(rx)) > 1) {
            TargetFrontLeftPower /= power + Math.abs(rx);
            TargetFrontRightPower /= power + Math.abs(rx);
            TargetRearLeftPower /= power + Math.abs(rx);
            TargetRearRightPower /= power + Math.abs(rx);
        }

        // Apply ramping
        CurrentFrontLeftPower = getRampedPower(CurrentFrontLeftPower, TargetFrontLeftPower);
        CurrentRearLeftPower = getRampedPower(CurrentRearLeftPower, TargetRearLeftPower);
        CurrentFrontRightPower = getRampedPower(CurrentFrontRightPower, TargetFrontRightPower);
        CurrentRearRightPower = getRampedPower(CurrentRearRightPower, TargetRearRightPower);

        // Set motor powers
        FrontLeftMotor.setPower(CurrentFrontLeftPower);
        FrontRightMotor.setPower(CurrentFrontRightPower);
        RearLeftMotor.setPower(CurrentRearLeftPower);
        RearRightMotor.setPower(CurrentRearRightPower);

    }

    private double  getRampedPower(double currentPower, double targetPower) {
        // Calculate the error amount (how far we are from the target)
        double error = targetPower - currentPower;

        // Calculate the change
        // Math.min(error, RAMP_RATE) handles positive error
        // Math.max(..., -RAMP_RATE) handles negative error
        double change = error*Config.RAMP_RATE;

        currentPower += change;

        currentPower = Math.min(Config.MaxDriveSpeed, Math.max(currentPower, -Config.MaxDriveSpeed));

        // Return new power
        return currentPower;
    }

    public void autoInit(HardwareMap hardwareMap) {

        // calculate ticks per revolutions and per inch
        this.ticksPerWheelRev = (int)(Config.TICKS_PER_MOTOR_REV * Config.GEAR_RATIO);
        this.ticksPerInch = (int)(ticksPerWheelRev/Config.WHEEL_CIRCUMFERENCE);

        this.turnCircumference = 3.14 * Config.WHEEL_BASE_WIDTH;

        // get and set all the actual hardware for each motor
        FrontLeftMotor = hardwareMap.get(DcMotor.class, Config.FRONT_LEFT_DRIVE_NAME);
        RearLeftMotor = hardwareMap.get(DcMotor.class, Config.REAR_LEFT_DRIVE_NAME);
        FrontRightMotor = hardwareMap.get(DcMotor.class, Config.FRONT_RIGHT_DRIVE_NAME);
        RearRightMotor = hardwareMap.get(DcMotor.class, Config.REAR_RIGHT_DRIVE_NAME);

        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        FrontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        RearRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // IMU setup

        RevHubOrientationOnRobot hubOrientation = new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
        );

        IMU.Parameters parameters = new IMU.Parameters(hubOrientation);

        // Get imu from hardware map
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(parameters);

        imu.resetYaw();

    }

    public void moveForward(double distance) {

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int)(distance * (double)this.ticksPerInch);

        FrontRightMotor.setTargetPosition(targetPosition);
        FrontLeftMotor.setTargetPosition(targetPosition);
        RearRightMotor.setTargetPosition(targetPosition);
        RearLeftMotor.setTargetPosition(targetPosition);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (FrontRightMotor.isBusy() || FrontLeftMotor.isBusy() || RearRightMotor.isBusy() || RearLeftMotor.isBusy()) {
            double power = getPower(targetPosition);

            FrontRightMotor.setPower(power);
            FrontLeftMotor.setPower(power);
            RearRightMotor.setPower(power);
            RearLeftMotor.setPower(power);

        }

        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        RearRightMotor.setPower(0);
        RearLeftMotor.setPower(0);

    }


    public void moveBackward(double distance) {

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int)(distance * (double)this.ticksPerInch);

        FrontRightMotor.setTargetPosition(-targetPosition);
        FrontLeftMotor.setTargetPosition(-targetPosition);
        RearRightMotor.setTargetPosition(-targetPosition);
        RearLeftMotor.setTargetPosition(-targetPosition);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (FrontRightMotor.isBusy() || FrontLeftMotor.isBusy() || RearRightMotor.isBusy() || RearLeftMotor.isBusy()) {
            double power = getPower(targetPosition);

            FrontRightMotor.setPower(power);
            FrontLeftMotor.setPower(power);
            RearRightMotor.setPower(power);
            RearLeftMotor.setPower(power);

        }

        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        RearRightMotor.setPower(0);
        RearLeftMotor.setPower(0);

    }

    public void strafeLeft(double distance) {

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int)((ticksPerWheelRev/12.0)*distance);

        FrontRightMotor.setTargetPosition(-targetPosition);
        FrontLeftMotor.setTargetPosition(targetPosition);
        RearRightMotor.setTargetPosition(targetPosition);
        RearLeftMotor.setTargetPosition(-targetPosition);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (FrontRightMotor.isBusy() || FrontLeftMotor.isBusy() || RearRightMotor.isBusy() || RearLeftMotor.isBusy()) {
            double power = getPower(targetPosition)*0.6;

            FrontRightMotor.setPower(-power);
            FrontLeftMotor.setPower(power);
            RearRightMotor.setPower(-power);
            RearLeftMotor.setPower(power);

        }

        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        RearRightMotor.setPower(0);
        RearLeftMotor.setPower(0);
    }

    public void strafeRight(double distance) {

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        int targetPosition = (int)((ticksPerWheelRev/12.0)*distance);

        FrontRightMotor.setTargetPosition(targetPosition);
        FrontLeftMotor.setTargetPosition(-targetPosition);
        RearRightMotor.setTargetPosition(-targetPosition);
        RearLeftMotor.setTargetPosition(targetPosition);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        FrontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        while (FrontRightMotor.isBusy() || FrontLeftMotor.isBusy() || RearRightMotor.isBusy() || RearLeftMotor.isBusy()) {
            double power = getPower(targetPosition)*0.6;

            FrontRightMotor.setPower(-power);
            FrontLeftMotor.setPower(power);
            RearRightMotor.setPower(-power);
            RearLeftMotor.setPower(power);

        }

        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        RearRightMotor.setPower(0);
        RearLeftMotor.setPower(0);
    }
    public void turnDegrees(int degrees) {

        double arc_length = ((double)degrees / 360.0) * turnCircumference;
        int turn_ticks = (int)(arc_length * ticksPerInch);

        FrontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        FrontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        FrontRightMotor.setTargetPosition(-turn_ticks);
        FrontLeftMotor.setTargetPosition(turn_ticks);
        RearRightMotor.setTargetPosition(-turn_ticks);
        RearLeftMotor.setTargetPosition(turn_ticks);

        FrontRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (FrontRightMotor.isBusy() || FrontLeftMotor.isBusy() || RearRightMotor.isBusy() || RearLeftMotor.isBusy()) {
            double power = getPower(turn_ticks) * 2;

            FrontRightMotor.setPower(-power);
            FrontLeftMotor.setPower(power);
            RearRightMotor.setPower(-power);
            RearLeftMotor.setPower(power);

        }

        FrontRightMotor.setPower(0);
        FrontLeftMotor.setPower(0);
        RearRightMotor.setPower(0);
        RearLeftMotor.setPower(0);

    }

    public void turnTo(double degrees) {

        imu.resetYaw();

        // --- Controller Constants (you will need to tune these) ---
        final double P_GAIN = 0.02;      // How fast to react
        final double HEADING_TOLERANCE = 1.0; // How close is "close enough"
        final double MIN_POWER = 0.15;   // Minimum power to overcome friction
        final double MAX_POWER = 0.8;    // Maximum power to apply

        degrees -= 10;

        // Set motors to run without encoders for turning
        FrontLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        FrontRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearLeftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RearRightMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        FrontRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        RearRightMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        FrontLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RearLeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        // Changed this loop from opModeIsActive() to true.
        // The "break" statement (further down) is now the only way
        // to exit the loop.
        while (true) {

            // 1. Get the robot's current heading
            double currentYaw = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);

            // 2. Calculate the error
            double error = degrees - currentYaw;

            // Handle the "wrap-around" (e.g., turning from -170 to +170)
            if (error > 180) {
                error -= 360;
            } else if (error < -180) {
                error += 360;
            }

            // 3. Check if we've reached the target
            if (Math.abs(error) < HEADING_TOLERANCE) {
                break; // Exit the loop
            }

            // 4. Calculate motor power
            double power = P_GAIN * error;

            // 5. Constrain the power
            if (Math.abs(power) < MIN_POWER) {
                power = Math.signum(power) * MIN_POWER;
            }
            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            // 6. Apply power to the motors
            FrontLeftMotor.setPower(-power);
            RearLeftMotor.setPower(-power);
            FrontRightMotor.setPower(power);
            RearRightMotor.setPower(power);
        }

        // 7. Stop all motors
        FrontLeftMotor.setPower(0);
        FrontRightMotor.setPower(0);
        RearLeftMotor.setPower(0);
        RearRightMotor.setPower(0);

    }

    private double getPower(int targetPosition) {
        int currentPosition = FrontRightMotor.getCurrentPosition();

        int remainingDistance = targetPosition - currentPosition;

        double power;

        // acceleration phase
        if (currentPosition < Config.ACCELERATION_DISTANCE) {
            power = ( (double)currentPosition / Config.ACCELERATION_DISTANCE) * Config.AUTO_MAX_SPEED;
        }
        // deceleration phase
        else if (remainingDistance < Config.DECELERATION_DISTANCE) {
            power = ( (double)remainingDistance / Config.ACCELERATION_DISTANCE) * Config.AUTO_MAX_SPEED;
        }
        // cruise phase
        else {
            power = Config.AUTO_MAX_SPEED;
        }

        // prevents stalling
        if (power < 0.1) { power = 0.1; }
        return power;
    }


}
