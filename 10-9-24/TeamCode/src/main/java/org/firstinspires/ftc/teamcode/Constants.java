package org.firstinspires.ftc.teamcode;

public class Constants
{
    public enum Team
    {
        Undetermined,
        Red,
        Blue
    }

    // +++ I forgot to tell you. But for naming Static Final stuff, I use All Caps Camel Case. Which looks like this LA_LA_LA, or HIGH_BUCKET, or CLAW_CLOSED
    public final static int lalala= 2070;
    public final static int highBucket = 5440;
    public final static int lowBucket  = 2680;
    public final static int lowSample = 980;
    public final static int highSample = 2640;
    public final static int clawclosed =0; // +++ this should at least have a Capital C
    public final static int clawOpen = 0;
    public final static int intakeLiftMotorUp = 120;
    public final static int intakeLiftMotorDown = 750;

    public static Team CURRENT_TEAM = Team.Undetermined;

    // =================================== INTAKE ALGORITHM ===================================
    public final static short RED_RANGE_MIN = 0;
    public final static short RED_RANGE_MAX = 20;

    public final static short BLUE_RANGE_MIN = 170;
    public final static short BLUE_RANGE_MAX = 270;

    public final static double INTAKE_TIME = 0.5;



}
