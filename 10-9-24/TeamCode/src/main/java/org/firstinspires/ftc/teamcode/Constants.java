package org.firstinspires.ftc.teamcode;

public class Constants
{
    public enum Team
    {
        Undetermined,
        Red,
        Blue
    }

    public final static int HIGH_BUCKET = 5440 + 75;
    public final static int LOW_BUCKET = 2780;
    public final static int LOW_SAMPLE = 980;
    public final static int HIGH_SAMPLE = 2640;
    public final static int winchUp = 0;
    public final static int winchDown = 0;
    public final static int elavatorDown = 0;
    public final static int slideMin = 0;
    public final static int clawOpen = 0;
    public final static int clawClosed = 0;
    public final static int clawBucket = 0;
    public final static int clawSample = 0;
    public final static int clawPickUp = 0;
    public final static int clawgrab = 0;


    public static class DriveConstants
    {
        public final static float DEAD_WHEEL_DIAMETER = 1; // All units are in mm
        public final static float ODOMETER_TICKS_PER_ROTATION = 104; // All units are in mm
        public final static float X_DISTANCE_FROM_CENTER = 104; // All units are in mm
        public final static float Z_DISTANCE_FROM_CENTER = 104; // All units are in mm


        public final static float WHEEL_DIAMETER = 104; // All units are in mm





        // ======================================================== DO NOT EDIT ========================================================
        public final static float ODOMETER_DISTANCE_PER_TICK = (1 / ODOMETER_TICKS_PER_ROTATION) * DEAD_WHEEL_DIAMETER; // All units are in mm

    }

    public static class ClawConstants
    {
        public final static double ARM_SERVO_ROTATION_AMOUNT = 360 * 5;
        public final static double WRIST_SERVO_ROTATION_AMOUNT = 300;
        public final static double CLAW_SERVO_ROTATION_AMOUNT = 300;



        public final static double CLAW_CLOSE_DEGREES = 0;
        public final static double CLAW_OPEN_DEGREES = 55;


        public final static double WRIST_DEGREES_PER_SECOND = 115 / 60.0; // 115 RPM / 60 Seconds

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
