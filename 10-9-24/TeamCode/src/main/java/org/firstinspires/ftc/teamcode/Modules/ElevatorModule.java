package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

public class ElevatorModule
{

    private DcMotor leftElevatorMotor;
    private DcMotor rightElevatorMotor;


    private ElevatorModule(DcMotor leftElevatorMotor, DcMotor rightElevatorMotor)
    {
        this.leftElevatorMotor = leftElevatorMotor;
        this.rightElevatorMotor = rightElevatorMotor;
    }


    public void Start()
    {

    }

    public void Update(double deltaTime)
    {

    }

    public void Stop()
    {

    }


}
