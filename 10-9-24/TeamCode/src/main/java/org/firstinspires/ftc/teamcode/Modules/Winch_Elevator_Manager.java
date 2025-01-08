package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.Timer;

public class Winch_Elevator_Manager implements Runnable
{


    private boolean isAlive;

    // Modules
    public WinchModule winchModule;
    public ElevatorModule elevatorModule;

    private Timer timer;

    private double winchDegrees;
    private double elevatorPosition;
    private boolean isControllingMovement = false;
    private boolean isFirstFrameContollingMovement = false;

    public Winch_Elevator_Manager(HardwareMap hardwareMap)
    {
        winchModule = new WinchModule(hardwareMap.get(DcMotor.class, Constants.HardwareConstants.LEFT_WINCH_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.RIGHT_WINCH_MOTOR_NAME));
        elevatorModule = new ElevatorModule(hardwareMap.get(DcMotor.class, Constants.HardwareConstants.LEFT_ELEVATOR_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.RIGHT_ELEVATOR_MOTOR_NAME));


        timer = new Timer();
    }


    @Override
    public void run()
    {
        isAlive = true;
        timer.Reset();
        winchModule.Start();
        elevatorModule.Start();

        while (isAlive)
        {
            timer.Update();

            winchModule.Update(timer.deltaTime);
            elevatorModule.Update(timer.deltaTime);

            if (!isFirstFrameContollingMovement)
            {
                if (winchModule.GetTargetDegrees() != winchDegrees || elevatorModule.GetTargetPosition() != elevatorPosition)
                {
                    isControllingMovement = false;
                }
            }
            if (isControllingMovement)
            {
                if (elevatorModule.GetPosition() < 100)
                {
                    winchModule.SetTargetDegrees(winchDegrees);
                }
                else if (winchModule.GetTargetDegrees() - winchModule.GetDegrees() < 5)
                {
                    winchModule.SetTargetDegrees(winchDegrees);
                }
                else
                {
                    elevatorModule.SetPosition(50);
                }

                if (winchModule.GetTargetDegrees() - winchModule.GetDegrees() < 5)
                {
                    elevatorModule.SetPosition(elevatorPosition);
                }
            }



            if ((elevatorModule.GetTargetPosition() - elevatorModule.GetPosition()) < 0)
            {
                elevatorModule.SetPower(0.5);
            }
            else
            {
                elevatorModule.SetPower(1);
            }

        }

        winchModule.Stop();
        elevatorModule.Stop();
    }



    public void Stop()
    {
        isAlive = false;
        winchModule.Stop();
    }



    public void MoveWinch_And_Elevator(double winchDegrees, double elevatorPosition)
    {
        isFirstFrameContollingMovement = true;
        isControllingMovement = true;
        this.winchDegrees = winchDegrees;
        this.elevatorPosition = elevatorPosition;
    }

}
