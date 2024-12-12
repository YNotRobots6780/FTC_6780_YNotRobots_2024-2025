package org.firstinspires.ftc.teamcode.modules;

import org.firstinspires.ftc.teamcode.Constants;

public class PathfindingModule {

    private int lastLeftTick;
    private int lastRightTick;
    private int lastBackTick;


    private double x;
    private double z;
    private double rotation;
    private double rotationInRadians;


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
        double xDelta;
        double zDelta;
        double leftDelta = (HardwareModule.leftOdometer.getCurrentTicks() - lastLeftTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;
        double rightDelta = (HardwareModule.rightOdometer.getCurrentTicks() - lastRightTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;
        double backDelta = (HardwareModule.backOdometer.getCurrentTicks() - lastBackTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;

        lastLeftTick = HardwareModule.leftOdometer.getCurrentTicks();
        lastRightTick = HardwareModule.rightOdometer.getCurrentTicks();
        lastBackTick = HardwareModule.backOdometer.getCurrentTicks();

        double rotationDelta = ((leftDelta - rightDelta) / (Constants.DriveConstants.X_DISTANCE_FROM_CENTER * 2));

        if (rotationDelta == 0)
        {
            xDelta = backDelta;
            zDelta = rightDelta;
        }
        else
        {
            xDelta = 2 * Math.sin(rotationDelta/2) * backDelta/rotationDelta + Constants.DriveConstants.Z_DISTANCE_FROM_CENTER;
            zDelta = rightDelta/rotationDelta + Constants.DriveConstants.X_DISTANCE_FROM_CENTER;
        }


        // Rotating the Movement from Robot-Centric to Field-Centric;
        double averageOrientation = Math.toDegrees(rotationInRadians + rotationDelta / 2);
        // x += (xDelta * Math.sin(Math.toDegrees(averageOrientation))) - (zDelta *;
        z += zDelta * Math.sin(Math.toDegrees(averageOrientation));






        rotationInRadians += rotationDelta;

        rotation = Math.toDegrees(rotationInRadians);
    }

    public void SetPosition(float x, float z, float rotation)
    {
        this.x = x;
        this.z = z;
        this.rotation = rotation;
    }

}
