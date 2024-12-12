package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.Constants;

public class SlideModule{

    // Settings
    private double distanceOffset;


    public SlideModule()
    {
        distanceOffset = Constants.SlideConstants.DISTANCE_OFFSET;
    }


    public void SetDistance(double millimeters)
    {
        int position = (int)Math.round((millimeters - distanceOffset) / Constants.SlideConstants.MILI_METERS_PER_ENCODER_TICK);
        HardwareModule.winchMotorLeft.setTargetPosition(position);
        HardwareModule.winchMotorRight.setTargetPosition(position);
    }

    public void SetDistanceOffset(double millimeters)
    {
        distanceOffset = millimeters;
    }





    public double GetTargetDistance()
    {
        return HardwareModule.winchMotorLeft.getTargetPosition() * Constants.SlideConstants.MILI_METERS_PER_ENCODER_TICK * distanceOffset;
    }

    public double GetCurrentDistance()
    {
        return HardwareModule.winchMotorLeft.getCurrentPosition() * Constants.SlideConstants.MILI_METERS_PER_ENCODER_TICK * distanceOffset;
    }

    public void SetSpeed(double speed)
    {
        HardwareModule.winchMotorLeft.setPower(speed);
        HardwareModule.winchMotorRight.setPower(speed);
    }

    public double GetSpeed()
    {
        return HardwareModule.winchMotorLeft.getPower();
    }



}
