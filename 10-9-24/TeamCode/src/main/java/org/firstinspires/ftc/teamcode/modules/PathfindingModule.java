package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;
import org.firstinspires.ftc.teamcode.Constants;

public class PathfindingModule {

    private int lastLeftTick;
    private int lastRightTick;
    private int lastBackTick;


    private double x;
    private double z;
    private double rotation;


    public void ResetPosition()
    {
        lastLeftTick = HardwareModule.leftOdometer.getCurrentTicks();
        lastRightTick = HardwareModule.rightOdometer.getCurrentTicks();
        lastBackTick = HardwareModule.backOdometer.getCurrentTicks();

        x = 0;
        z = 0;
        rotation = 0;
    }

    public void Update()
    {
        // Units = mm
        double leftDelta = (HardwareModule.leftOdometer.getCurrentTicks() - lastLeftTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;
        double rightDelta = (HardwareModule.rightOdometer.getCurrentTicks() - lastRightTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;
        double backDelta = (HardwareModule.backOdometer.getCurrentTicks() - lastBackTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;

        lastLeftTick = HardwareModule.leftOdometer.getCurrentTicks();
        lastRightTick = HardwareModule.rightOdometer.getCurrentTicks();
        lastBackTick = HardwareModule.backOdometer.getCurrentTicks();

        double rotationDelta = ((leftDelta - rightDelta) / (Constants.DriveConstants.X_DISTANCE_FROM_CENTER * 2));

        if (rotationDelta == 0)
        {
            // x =
        }

    }

    public void SetPosition(float x, float z, float rotation)
    {
        this.x = x;
        this.z = z;
        this.rotation = rotation;
    }

}
