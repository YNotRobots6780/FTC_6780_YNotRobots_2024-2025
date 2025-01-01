package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.PathfindingModule;
import org.firstinspires.ftc.teamcode.core.Encoder;

public class Robot
{


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

    public void Start()
    {
        pathfindingModule.ResetPosition();
        pathfindingThread.start();
    }

    public void Update()
    {

    }

    public void Stop()
    {
        pathfindingModule.Stop();
    }
}
