package org.firstinspires.ftc.teamcode;

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

    }

    public static class DriveConstants
    {
        public final static float DEAD_WHEEL_DIAMETER = 48; // All units are in mm
        public final static float ODOMETER_TICKS_PER_ROTATION = 2000;
        public final static float X_DISTANCE_FROM_CENTER = 182.5f; // All units are in mm
        public final static float Z_DISTANCE_FROM_CENTER = 168; // All units are in mm


        public final static float WHEEL_DIAMETER = 104; // All units are in mm


        public final static float PATH_FINDING_CLOSE_ENOUGH_ZONE = 25.4f; // All units are in mm

        // ======================================================== DO NOT EDIT ========================================================

        public final static double ODOMETER_REVOLUTIONS_PER_TICK = (1 / ODOMETER_TICKS_PER_ROTATION);
        public final static double DEAD_WHEEL_CIRCUMFERENCE = DEAD_WHEEL_DIAMETER * Math.PI;
        public final static double ODOMETER_DISTANCE_PER_TICK = (ODOMETER_REVOLUTIONS_PER_TICK * DEAD_WHEEL_CIRCUMFERENCE); // All units are in mm

    }

    public static class ClawConstants
    {
        public final static double WRIST_SERVO_ROTATION_AMOUNT = 300;
        public final static double CLAW_SERVO_ROTATION_AMOUNT = 300;
        public final static double ARM_SERVO_ROTATION_AMOUNT = 300;


        public final static double CLAW_CLOSE_DEGREES = 240;
        public final static double CLAW_OPEN_DEGREES = 300;

        public final static double WRIST_DEFAULT_POSITION_DEGREES = 135;
        public final static double WRIST_OFFSET_POSITION_DEGREES = 35;

        public final static double ARM_STRAIGHT_UP_POSITION_DEGREES = 0;
        public final static double ARM_STRAIGHT_OUT_POSITION_DEGREES = 90;
        public final static double ARM_STRAIGHT_DOWN_POSITION_DEGREES = 180;
        public final static double ARM_HALF_DOWN_POSITION_DEGREES = 135;
    }

    public static class SlideConstants
    {
        public final static double DISTANCE_OFFSET = 50; // mm
        public final static double ENCODER_TICKS_PER_ROTATION = 384.5; // mm
        public final static double OUTPUT_PER_ROTATION = 17.825; // mm
        public final static double MILI_METERS_PER_ENCODER_TICK = OUTPUT_PER_ROTATION / ENCODER_TICKS_PER_ROTATION; // mm
    }

    public static class WinchConstants
    {
        public final static double ANGLE_OFFSET = 50; // mm
        public final static double ENCODER_TICKS_PER_ROTATION = 537.7; // mm
        public final static double OUTPUT_PER_ROTATION = 57; // mm
        public final static double MILI_METERS_PER_ENCODER_TICK = OUTPUT_PER_ROTATION / ENCODER_TICKS_PER_ROTATION; // mm
    }
}
