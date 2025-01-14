package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.Drive_Claw_Manager;
import org.firstinspires.ftc.teamcode.Modules.PathfindingModule;
import org.firstinspires.ftc.teamcode.Modules.Winch_Elevator_Manager;
import org.firstinspires.ftc.teamcode.core.Encoder;

public class Robot
{


    public PathfindingModule pathfindingModule;
    private Thread pathfindingThread;

    public Drive_Claw_Manager drive_claw_manager;
    private Thread drive_claw_thread;

    public Winch_Elevator_Manager winch_elevator_manager;
    private Thread winch_elevator_thread;

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



        drive_claw_manager = new Drive_Claw_Manager(hardwareMap);

        drive_claw_thread = new Thread(drive_claw_manager, "Drive & Claw Thread");



        winch_elevator_manager = new Winch_Elevator_Manager(hardwareMap);

        winch_elevator_thread = new Thread(winch_elevator_manager, "Winch & Elevator Thread");
    }
    public Robot(HardwareMap hardwareMap, Telemetry telemetry)
    {
        System.out.println("Hello, World!");

        pathfindingModule = new PathfindingModule(
                new Encoder(hardwareMap.get(DcMotor.class, "front_left")),
                new Encoder(hardwareMap.get(DcMotor.class, "front_right")),
                new Encoder(hardwareMap.get(DcMotor.class, "back_left")));

        pathfindingThread = new Thread(pathfindingModule, "pathfinding Thread");



        drive_claw_manager = new Drive_Claw_Manager(hardwareMap);

        drive_claw_thread = new Thread(drive_claw_manager, "Drive & Claw Thread");



        winch_elevator_manager = new Winch_Elevator_Manager(hardwareMap);

        winch_elevator_thread = new Thread(winch_elevator_manager, "Winch & Elevator Thread");
    }

    public void Start()
    {
        pathfindingModule.ResetPosition();
        pathfindingThread.start();

        drive_claw_thread.start();

        winch_elevator_thread.start();
    }

    public void Update()
    {

    }

    public void Stop()
    {
        pathfindingModule.Stop();
        drive_claw_manager.Stop();
        winch_elevator_manager.Stop();
    }



    public void ScoreSample()
    {

    }

    public void ScoreSpecimen()
    {

    }
}
