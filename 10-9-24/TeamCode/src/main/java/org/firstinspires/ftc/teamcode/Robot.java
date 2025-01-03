package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Modules.Drive_Claw_Manager;
import org.firstinspires.ftc.teamcode.Modules.PathfindingModule;
import org.firstinspires.ftc.teamcode.core.Encoder;

public class Robot
{


    private PathfindingModule pathfindingModule;
    private Thread pathfindingThread;

    // private final Drive_Claw_Manager driveClawManager;
    // private final Thread driveClawThread;

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



        // driveClawManager = new Drive_Claw_Manager(hardwareMap);

        // driveClawThread = new Thread(driveClawManager, "Drive & Claw Thread");
    }

    public void Start()
    {
        pathfindingModule.ResetPosition();
        pathfindingThread.start();

        // driveClawThread.start();
    }

    public void Update()
    {

    }

    public void Stop()
    {
        pathfindingModule.Stop();
        // driveClawManager.Stop();
    }
}
