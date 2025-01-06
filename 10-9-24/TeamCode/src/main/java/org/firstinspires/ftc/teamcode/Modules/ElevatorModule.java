package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

public class ElevatorModule
{

    private DcMotor leftElevatorMotor;
    private DcMotor rightElevatorMotor;

    private int targetPosition;
    private double power;

    private boolean shouldUpdatePosition;
    private boolean shouldUpdatePower;


    public ElevatorModule(DcMotor leftElevatorMotor, DcMotor rightElevatorMotor)
    {
        this.leftElevatorMotor = leftElevatorMotor;
        this.rightElevatorMotor = rightElevatorMotor;
    }


    public void Start()
    {
        leftElevatorMotor.setTargetPosition(leftElevatorMotor.getCurrentPosition());
        rightElevatorMotor.setTargetPosition(rightElevatorMotor.getCurrentPosition());
        leftElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightElevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftElevatorMotor.setPower(0);
        rightElevatorMotor.setPower(0);
    }

    public void Update(double deltaTime)
    {
        if (shouldUpdatePosition)
        {
            shouldUpdatePosition = false;
            leftElevatorMotor.setTargetPosition(targetPosition);
            rightElevatorMotor.setTargetPosition(targetPosition);
        }

        if (shouldUpdatePower)
        {
            shouldUpdatePower = false;
            leftElevatorMotor.setPower(power);
            rightElevatorMotor.setPower(power);
        }
    }

    public void Stop()
    {
        leftElevatorMotor.setPower(0);
        rightElevatorMotor.setPower(0);
    }


    public void SetPosition(double milimeters)
    {
        shouldUpdatePosition = true;
        milimeters = ClampExtention(milimeters);
        targetPosition = (int)Math.round((milimeters - Constants.ElevatorConstants.EXTENSION_OFFSET) / Constants.ElevatorConstants.DISTANCE_PER_ENCODER_TICK);
    }
    public double GetPosition()
    {
        return (leftElevatorMotor.getCurrentPosition() * Constants.ElevatorConstants.DISTANCE_PER_ENCODER_TICK) + Constants.ElevatorConstants.EXTENSION_OFFSET;
    }
    public double GetTargetPosition()
    {
        return (targetPosition * Constants.ElevatorConstants.DISTANCE_PER_ENCODER_TICK) + Constants.ElevatorConstants.EXTENSION_OFFSET;
    }

    public void SetPower(double power)
    {
        shouldUpdatePower = true;
        this.power = power;
    }
    public double GetPower()
    {
        return power;
    }



    private double ClampExtention(double value)
    {
        if (value > Constants.ElevatorConstants.MAX_EXTENSION)
        {
            return Constants.ElevatorConstants.MAX_EXTENSION;
        }
        else if (value < Constants.ElevatorConstants.MIN_EXTENSION)
        {
            return Constants.ElevatorConstants.MIN_EXTENSION;
        }
        else
        {
            return value;
        }
    }

}
