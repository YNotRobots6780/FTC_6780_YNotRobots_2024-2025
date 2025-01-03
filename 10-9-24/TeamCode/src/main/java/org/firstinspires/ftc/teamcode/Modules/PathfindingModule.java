package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.Encoder;

public class PathfindingModule implements Runnable {

    private static PathfindingModule instance;
    public static PathfindingModule Instance()
    {
        if (instance == null)
        {
            System.err.println("NO INSTANCE OF PATHFINDING MODULE YET CREATED!!!!!!!\n" + Thread.currentThread().getStackTrace());
        }
        return instance;
    }

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
        /*
        if (instance != null)
        {
            this.leftOdometer = null;
            this.rightOdometer = null;
            this.backOdometer = null;
            System.err.println("TWO INSTANCES OF THE PATHFINDING MODULE!!!!\n" + Thread.currentThread().getStackTrace());
            return;
        }
        instance = this;
*/
        leftOdometer.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightOdometer.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backOdometer.SetMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftOdometer.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightOdometer.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backOdometer.SetMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftOdometer.SetReverse(true);
        rightOdometer.SetReverse(true);


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
                zDelta = (rightDelta + leftDelta) / 2;
            }
            else
            {
                // B - B  * roation
                //      x
                xDelta = backDelta - (Constants.DriveConstants.Z_DISTANCE_FROM_CENTER * rotationDelta);
                zDelta = (rightDelta + leftDelta) / 2;
            }


            // Rotating the Movement from Robot-Centric to Field-Centric;
            double averageOrientation = (rotationInRadians + (rotationDelta / 2));
            x += (xDelta * Math.cos(averageOrientation)) - (zDelta * Math.sin(averageOrientation));
            z += (xDelta * Math.sin(averageOrientation)) + (zDelta * Math.cos(averageOrientation));

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
