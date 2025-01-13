package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

@Config
public class Constants
{
    public enum Team
    {
        Undetermined,
        Red,
        Blue
    }

    public final static double DRIVE_SPEED = 1;
    public final static double SLOW_DRIVE_SPEED = 0.25;

    public final static int ELEVATOR_HIGH_CHAMBER = 500;
    public final static int ELEVATOR_LOW_CHAMBER = 0;
    public final static int ELEVATOR_HIGH_BASKET = 1750;
    public final static int ELEVATOR_LOW_BASKET = 650;

    public final static int WINCH_HIGH_CHAMBER = 625;
    public final static int WINCH_LOW_CHAMBER = 225;
    public final static int WINCH_HIGH_BASKET = 675;
    public final static int WINCH_LOW_BASKET = 600;

    public final static double GRAB_WINCH_TO_ELEVATOR_RATIO = 1 / 40.0;


    public final static double WRIST_MODIFIER = (0.3 - ((1 - 0.1) / 0.9 * 0.3));
    public final static double WRIST_DEFAULT_POSITION = 0;

    public final static double armUp = 0;
    public final static double armPickUp= 0.5;
    public final static double armStraightDown = 0.6;
    public final static double armMid= 0.25;

    public final static double CLAW_CLOSE_POSITION = 0.8;
    public final static double CLAW_OPEN_POSITION = 1;


    public final static double TARGET_PICKUP_TIME = 1; // second

    public final static double WINCH_SPEED = 0.2;

    public static class HardwareConstants
    {
        public final static String FRONT_LEFT_DRIVE_MOTOR_NAME = "front_left"; // EX: 0
        public final static String FRONT_RIGHT_DRIVE_MOTOR_NAME = "front_right"; // 3
        public final static String BACK_LEFT_DRIVE_MOTOR_NAME = "back_left"; // EX: 1
        public final static String BACK_RIGHT_DRIVE_MOTOR_NAME = "back_right"; // 2


        public final static String RIGHT_ELEVATOR_MOTOR_NAME = "elevatorMotorRight"; // 0
        public final static String LEFT_ELEVATOR_MOTOR_NAME = "elevatorMotorLeft"; // EX: 3


        public final static String RIGHT_WINCH_MOTOR_NAME = "winchMotorRight"; // 1
        public final static String LEFT_WINCH_MOTOR_NAME = "winchMotorLeft"; // EX: 2


        public final static String RIGHT_ARM_SERVO_NAME = "armServoRight"; //1
        public final static String LEFT_ARM_SERVO_NAME = "armServoLeft"; //2
        public final static String CLAW_SERVO_NAME = "clawServo"; //3
        public final static String WRIST_SERVO_NAME = "wristServo"; //0

        public final static String TOP_COLOR_SENSOR = "topColorSensor"; //0
        public final static String BOTTOM_COLOR_SENSOR = "bottomColorSensor"; //0


        public final static String LEFT_ODOMETER_NAME = "front_left"; // EX: 0
        public final static String RIGHT_ODOMETER_NAME = "front_right"; // 3
        public final static String BACK_ODOMETER_NAME = "back_left"; // EX: 1

        public final static String IMU_NAME = "imu";

    }

    @Config
    public static class PathfindingModule
    {
        public final static float DEAD_WHEEL_DIAMETER = 48; // All units are in mm
        public final static float ODOMETER_TICKS_PER_ROTATION = 2000;
        public final static float X_DISTANCE_FROM_CENTER = 368/2.0f; // All units are in mm
        public final static float Z_DISTANCE_FROM_CENTER = 168; // All units are in mm


        // ======================================================== DO NOT EDIT ========================================================

        public final static double ODOMETER_REVOLUTIONS_PER_TICK = (1 / ODOMETER_TICKS_PER_ROTATION);
        public final static double DEAD_WHEEL_CIRCUMFERENCE = DEAD_WHEEL_DIAMETER * Math.PI;
        public final static double ODOMETER_DISTANCE_PER_TICK = (ODOMETER_REVOLUTIONS_PER_TICK * DEAD_WHEEL_CIRCUMFERENCE); // All units are in mm
    }

    @Config
    public static class DriveConstants
    {
        public final static double TURNING_KP = 0.01;
        public final static double TURNING_MAX_ERROR = 60;
        public final static double TURNING_KI = 0.01;
        public final static double TURNING_KI_ACTIVE_ZONE = 10;
        public final static double TURNING_KD = 0.01;
        public final static double TURNING_DEAD_ZONE = 2.5;
    }

    @Config
    public static class IMUConstants
    {
        private static final RevHubOrientationOnRobot.LogoFacingDirection LOGO_DIRECTION = RevHubOrientationOnRobot.LogoFacingDirection.DOWN;
        private static final RevHubOrientationOnRobot.UsbFacingDirection USB_DIRECTION = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;

        public static final RevHubOrientationOnRobot ROBOT_ORIENTATION = new RevHubOrientationOnRobot(LOGO_DIRECTION, USB_DIRECTION);
    }

    @Config
    public static class ClawConstants
    {
        public final static double WRIST_SERVO_ROTATION_AMOUNT = 300;
        public final static double CLAW_SERVO_ROTATION_AMOUNT = 300;
        public final static double ARM_SERVO_ROTATION_AMOUNT = 300;


        public final static double CLAW_CLOSE_DEGREES = 85;
        public final static double CLAW_OPEN_DEGREES = 150;

        public final static double WRIST_DEFAULT_POSITION_DEGREES = 135;

        public final static double ARM_STRAIGHT_UP_POSITION_DEGREES = 0;
        public final static double ARM_STRAIGHT_OUT_POSITION_DEGREES = 90;
        public final static double ARM_STRAIGHT_DOWN_POSITION_DEGREES = 180;
    }

    @Config
    public static class ElevatorConstants
    {
        public final static double ENCODER_TICKS_PER_ROTATION = 384.5;
        public final static double GEAR_RATIO = 35.0 / 25.0;
        public final static double ENCODER_TICKS_PER_OUTPUT_SHAFT_ROTATION = ENCODER_TICKS_PER_ROTATION / GEAR_RATIO;

        public final static double DISTANCE_PER_ROTATION = 17.825 * 2 * Math.PI; // mm
        public final static double DISTANCE_PER_ENCODER_TICK = DISTANCE_PER_ROTATION / ENCODER_TICKS_PER_OUTPUT_SHAFT_ROTATION;

        public final static double MAX_EXTENSION = 976;
        public final static double MIN_EXTENSION = 80;
        public final static double EXTENSION_OFFSET = 80;

    }

    @Config
    public static class WinchConstants
    {
        public final static double WINCH_OFFSET = -3.5;

        public final static double ENCODER_TICKS_PER_ROTATION = 1425.1;
        public final static double MILIMETERS_PER_ROTATION = 157; // mm

        public final static double MILIMETERS_PER_ENCODER_TICKS = MILIMETERS_PER_ROTATION / ENCODER_TICKS_PER_ROTATION;

        public final static double MINIMUM_WINCH_DEGREES = -3.5;
        public final static double MAXIMUM_WINCH_DEGREES = 55;
        public final static double STRING_TRAVEL_DISTANCE = 340; // mm

        public final static double DEGREES_PER_MILIMETERS = (MAXIMUM_WINCH_DEGREES - MINIMUM_WINCH_DEGREES) / STRING_TRAVEL_DISTANCE; // 0.1720588235294118
        public final static double DEGREES_PER_ENCODER_TICK = MILIMETERS_PER_ENCODER_TICKS * DEGREES_PER_MILIMETERS; // 0.0182394512575348


    }
}
