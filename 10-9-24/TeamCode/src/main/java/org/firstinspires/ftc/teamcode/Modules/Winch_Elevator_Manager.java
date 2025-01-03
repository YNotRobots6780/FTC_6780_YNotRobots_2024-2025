package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.Timer;

public class Winch_Elevator_Manager implements Runnable{


    private boolean isAlive;

    // Modules
    public WinchModule winchModule;
    public ElevatorModule elevatorModule;

    private Timer timer;



    public Winch_Elevator_Manager(HardwareMap hardwareMap)
    {
        winchModule = new WinchModule(hardwareMap.get(DcMotor.class, Constants.HardwareConstants.LEFT_WINCH_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.RIGHT_WINCH_MOTOR_NAME));
        // elevatorModule = new ElevatorModule();


        timer = new Timer();
    }


    @Override
    public void run()
    {
        isAlive = true;
        timer.Reset();
        winchModule.Start();

        while (isAlive)
        {
            timer.Update();

            winchModule.Update(timer.deltaTime);
            // elevatorModule.Update(timer.deltaTime);
        }
    }



    public void Stop()
    {
        isAlive = false;
        winchModule.Stop();
    }


}
