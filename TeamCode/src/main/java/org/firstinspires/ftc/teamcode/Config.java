package org.firstinspires.ftc.teamcode;

public class Config {

    // Teleop config variables
    public static final String FRONT_LEFT_DRIVE_NAME = "FrontLeftDrive";
    public static final String FRONT_RIGHT_DRIVE_NAME = "FrontRightDrive";
    public static final String REAR_LEFT_DRIVE_NAME = "RearLeftDrive";
    public static final String REAR_RIGHT_DRIVE_NAME = "RearRightDrive";

    public static final double MaxDriveSpeed = 0.9;
    public static final double RAMP_RATE = 0.05;

    // April tag related variables
    public static final String webcamName = "Webcam 1";
    public static final double aprilTagSize = 16.0; // Size in cm

    public static final double TURRET_DEADZONE = 0;

    public static final double KP = 0.05;

    // Example code
    public static final String angleAdjustServoName = "AngleAdjuster";
    public static final int angleAdjust = 20;
    public static final int defaultShooterServoAngle = 45;

    // Turret related variables
    public static final String turretMotorName = "turret";
    public static final String shootingServoName = "shooterServo";
    public static final String shootingMotorName = "shooterMotor";

    public static final double TICKS_PER_MOTOR_REV = 537.6;
    public static final double GEAR_RATIO = 1;
    public static final double WHEEL_CIRCUMFERENCE = 12.86;
    public static final double WHEEL_BASE_WIDTH = 14.2;
    public static final int    ACCELERATION_DISTANCE = 200;
    public static final int    DECELERATION_DISTANCE = 200;
    public static final double AUTO_MAX_SPEED = 0.5;
    public static final double TURRET_IDLE_TICKS = 100;

    // Intake related variables
    public static final String INTAKE_MOTOR_NAME = "intakeMotor";

}
