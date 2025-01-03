package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

public class WinchModule {


    private DcMotor leftElevatorMotor;
    private DcMotor rightElevatorMotor;


    public WinchModule(DcMotor leftElevatorMotor, DcMotor rightElevatorMotor)
    {
        this.leftElevatorMotor = leftElevatorMotor;
        this.rightElevatorMotor = rightElevatorMotor;


        this.leftElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightElevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
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



    public void SetDegrees(double degrees)
    {
        
    }

    public double GetDegrees()
    {
        return 0;
    }

    public void SetAngleOffset(double degrees)
    {

    }

    public double GetAngleOffset()
    {
        return 0;
    }

}
