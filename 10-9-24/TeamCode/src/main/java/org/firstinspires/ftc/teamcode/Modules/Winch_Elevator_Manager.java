package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.core.Timer;

public class Winch_Elevator_Manager implements Runnable{


    private boolean isAlive;

    // Modules
    private WinchModule winchModule;
    private ElevatorModule elevatorModule;

    private Timer timer;



    public Winch_Elevator_Manager(HardwareMap hardwareMap)
    {
        // winchModule = new ClawModule();
        // elevatorModule = new DriveModule();


        timer = new Timer();
    }


    @Override
    public void run()
    {
        isAlive = true;
        timer.Reset();

        while (isAlive)
        {
            timer.Update();

            winchModule.Update(timer.deltaTime);
            elevatorModule.Update(timer.deltaTime);
        }
    }



    public void Stop()
    {
        isAlive = false;
    }


}
