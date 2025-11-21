package org.firstinspires.ftc.teamcode.RobotClasses.Others;
/*
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Config;
import org.firstinspires.ftc.teamcode.RobotClasses.ModuleControllers.MG996RController;

import android.graphics.Color;

import java.util.Objects;

public class Indexer {

    private MG996RController servo;
    private ColorSensor colorSensor;
    private int[] balls = new int[] {0,0,0};
    private int nextBallInPattern = 0;
    private int currentSlot = 0;
    private int currentBallInPattern = 0;

    private ElapsedTime timer = new ElapsedTime();

    public Indexer(HardwareMap hardwareMap) {

        this.servo = new MG996RController(hardwareMap, Config.INDEXER_SERVO_NAME);

        this.colorSensor = hardwareMap.get(ColorSensor.class, Config.INDEXER_SENSOR_NAME);
        this.colorSensor.enableLed(true);

    }

    public void update(Telemetry telemetry, int[] pattern, Gamepad gamepad) {

        telemetry.addLine(getColorByName());

        String detectedColor = getColorByName();

        if (Objects.equals(detectedColor, "GREEN")) {
            balls[currentSlot] = 1;
        } else if (Objects.equals(detectedColor, "PURPLE")) {
            balls[currentSlot] = 2;
        }

        if (gamepad.x) {
            timer.reset();

            boolean rightColor = true;

            while (true) {

                updatePosition();

                if (balls[currentSlot] == pattern[nextBallInPattern]) {
                    break;
                }

            }
            unload();
            currentBallInPattern++;
            if (currentBallInPattern > 2) {
                currentBallInPattern = 0;
            }

        }

    }

    public void updatePosition() {
        if (timer.seconds() > Config.DURATION_PER_SIDE) {
            servo.setPower(1);
        } else {
            servo.setPower(0);
        }
    }

    public void unload() {
        if (currentSlot == 2) {
            balls[0] = 0;
        } else {
            balls[currentSlot + 1] = 0;
        }
    }

    public void resetPattern() {
        lastBallShot = -1;
        balls[0] = 0;
        balls[1] = 0;
        balls[2] = 0;
    }

    public String getColorByName() {
        int r = colorSensor.red();
        int g = colorSensor.green();
        int b = colorSensor.blue();

        float[] hsvValues = new float[3];

        Color.RGBToHSV(r * 8, g * 8, b * 8, hsvValues);

        float hue = hsvValues[0];

        if (getDistanceInCM() < 10.0) {
            if (hue > 60 && hue < 100) return "YELLOW";
            else if (hue > 110 && hue < 160) return "GREEN";
            else if (hue > 260 && hue < 320) return "PURPLE";
            else return "UNKNOWN";
        }
        else return "No balls";
    }

    public double getDistanceInCM() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM);
    }

}
*/