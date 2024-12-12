package org.firstinspires.ftc.teamcode.core;

public class MathF {
    public static double Clamp(double value, double min, double max)
    {
        if (value < min)
        {
            value = min;
        }
        else if (value > max)
        {
            value = max;
        }
        return value;
    }
}
