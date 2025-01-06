package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.Constants;

public class SlideModule{

    // Settings
    private double distanceOffset;


    public SlideModule()
    {
        distanceOffset = Constants.ElevatorConstants.DISTANCE_OFFSET;
    }


    public void SetDistance(double millimeters)
    {
        int position = (int)Math.round((millimeters - distanceOffset) / Constants.ElevatorConstants.MILI_METERS_PER_ENCODER_TICK);
        // HardwareModule.winchMotorLeft.setTargetPosition(position);
        // HardwareModule.winchMotorRight.setTargetPosition(position);
    }

    public void SetDistanceOffset(double millimeters)
    {
        distanceOffset = millimeters;
    }





    public double GetTargetDistance()
    {
         return 0;// HardwareModule.winchMotorLeft.getTargetPosition() * Constants.SlideConstants.MILI_METERS_PER_ENCODER_TICK * distanceOffset;
    }

    public double GetCurrentDistance()
    {
        return 0; // HardwareModule.winchMotorLeft.getCurrentPosition() * Constants.SlideConstants.MILI_METERS_PER_ENCODER_TICK * distanceOffset;
    }

    public void SetSpeed(double speed)
    {
        // HardwareModule.winchMotorLeft.setPower(speed);
        // HardwareModule.winchMotorRight.setPower(speed);
    }

    public double GetSpeed()
    {
        return 0;//HardwareModule.winchMotorLeft.getPower();
    }



}
