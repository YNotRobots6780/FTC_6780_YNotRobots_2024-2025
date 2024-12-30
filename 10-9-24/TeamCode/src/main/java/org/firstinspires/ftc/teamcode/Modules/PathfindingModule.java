package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.Encoder;

public class PathfindingModule implements Runnable {

    public boolean isAlive;


    private Encoder leftOdometer;
    private Encoder rightOdometer;
    private Encoder backOdometer;

    private int lastLeftTick;
    private int lastRightTick;
    private int lastBackTick;


    private double x;
    private double z;
    private double rotation;
    private double rotationInRadians;


    public PathfindingModule(Encoder leftOdometer, Encoder rightOdometer, Encoder backOdometer)
    {
        leftOdometer.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometer.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometer.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometer.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometer.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftOdometer.SetReverse(true);
        this.leftOdometer = leftOdometer;
        this.rightOdometer = rightOdometer;
        this.backOdometer = backOdometer;
    }

    public void ResetPosition()
    {
        lastLeftTick = leftOdometer.getCurrentTicks();
        lastRightTick = rightOdometer.getCurrentTicks();
        lastBackTick = backOdometer.getCurrentTicks();

        x = 0;
        z = 0;
        rotation = 0;
    }


    public void SetPosition(float x, float z, float rotation)
    {
        this.x = x;
        this.z = z;
        this.rotation = rotation;
    }


    @Override
    public void run() {
        isAlive = true;
        while (isAlive)
        {
            // Units = mm
            double xDelta;
            double zDelta;
            int rightEncoderValue = rightOdometer.getCurrentTicks();
            int leftEncoderValue = leftOdometer.getCurrentTicks();
            int backEncoderValue = backOdometer.getCurrentTicks();

            double leftDelta = (rightEncoderValue - lastLeftTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;
            double rightDelta = (leftEncoderValue - lastRightTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;
            double backDelta = (backEncoderValue - lastBackTick) * Constants.DriveConstants.ODOMETER_DISTANCE_PER_TICK;

            lastLeftTick = rightEncoderValue;
            lastRightTick = leftEncoderValue;
            lastBackTick = backEncoderValue;

            double rotationDelta = ((leftDelta - rightDelta) / (Constants.DriveConstants.X_DISTANCE_FROM_CENTER * 2));

            if (rotationDelta == 0)
            {
                xDelta = backDelta;
                zDelta = rightDelta;
            }
            else
            {
                xDelta = 2 * Math.sin(rotationDelta / 2 * (backDelta / rotationDelta + Constants.DriveConstants.Z_DISTANCE_FROM_CENTER));
                zDelta = rightDelta / rotationDelta + Constants.DriveConstants.X_DISTANCE_FROM_CENTER;
            }


            // Rotating the Movement from Robot-Centric to Field-Centric;
            double averageOrientation = Math.toDegrees(rotationInRadians + (rotationDelta / 2));
            x += (xDelta * Math.sin(averageOrientation));
            z += (zDelta * Math.cos(averageOrientation));

            rotationInRadians += rotationDelta;

            rotation = -Math.toDegrees(rotationInRadians);

            System.out.println("x: " + x);
            System.out.println("z: " + z);
            System.out.println("rotation: " + rotation);
        }
    }

    public void Stop()
    {
        isAlive = false;
    }
}
