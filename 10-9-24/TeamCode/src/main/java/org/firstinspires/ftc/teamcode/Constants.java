package org.firstinspires.ftc.teamcode;

public class Constants
{
    public enum Team
    {
        Undetermined,
        Red,
        Blue
    }

    public final static double ELEVATOR_POWER = 1;
    public final static int ELEVATOR_RETRACTED = 0;
    public final static int HIGH_BUCKET = 5440;
    public final static int LOW_BUCKET = 2680;
    public final static int LOW_SAMPLE = 980;
    public final static int HIGH_SAMPLE = 2640;

    public final static double CLAW_CLOSED = 2f/30; // 2mm out of 30 total
    public final static double CLAW_OPEN = 16f/30; // 6mm out of 30 total
    public final static double CLAW_GRAB = 6f/30; // 6mm out of 30 total

    public final static double INTAKE_LIFT_POWER = 1;
    public final static int INTAKE_LIFT_UP = 1300;
    public final static int INTAKE_LIFT_DOWN = 4870;

    public final static double INTAKE_POWER = 1;
    public final static double INTAKE_SERVO_POWER_FORWARD = 1;
    public final static double INTAKE_SERVO_POWER_BACKWARD = 0;
    public final static double INTAKE_SERVO_POWER_OFF = 0.5;
    public final static double OUTTAKE_POWER = 1;


    public static final double MOVEMENT_SPEED = 1;


    public static Team CURRENT_TEAM = Team.Undetermined;

    // =================================== INTAKE ALGORITHM ===================================
    public final static short RED_RANGE_MIN = 0;
    public final static short RED_RANGE_MAX = 20;

    public final static short BLUE_RANGE_MIN = 170;
    public final static short BLUE_RANGE_MAX = 270;

    public final static double INTAKE_TIME = 1;
    public final static double OUTTAKE_TIME = 0.5;

}
