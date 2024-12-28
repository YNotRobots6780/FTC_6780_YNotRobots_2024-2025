package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.PathfindingModule;
import org.firstinspires.ftc.teamcode.core.Encoder;

public class Robot implements Runnable
{

    public boolean isAlive;


    private final PathfindingModule pathfindingModule;
    private final Thread pathfindingThread;

    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor frontBackMotor;



    public Robot(HardwareMap hardwareMap)
    {
        System.out.println("Hello, World!");


        pathfindingModule = new PathfindingModule(
                new Encoder(hardwareMap.get(DcMotor.class, "front_left")),
                new Encoder(hardwareMap.get(DcMotor.class, "front_right")),
                new Encoder(hardwareMap.get(DcMotor.class, "back_left")));



        pathfindingThread = new Thread(pathfindingModule, "pathfinding Thread");

    }

    public void run()
    {
        // pathfindingModule.ResetPosition();
        pathfindingThread.start();
        // System.out.println("Multi Threading Starting!!");
        isAlive = true;
        while (isAlive)
        {
            // System.out.println("Hello, Multi Threading!!");
        }
    }




    public void Stop()
    {
        isAlive = false;
        pathfindingModule.Stop();
    }
}
