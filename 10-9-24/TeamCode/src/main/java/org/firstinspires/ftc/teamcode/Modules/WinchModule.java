package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Constants;

public class WinchModule {


    private double targetPosition = 0;
    private boolean shouldUpdatePosition;

    private double power = 1;
    private boolean shouldUpdatePower;


    private DcMotor leftWinchMotor;
    private DcMotor rightWinchMotor;


    public WinchModule(DcMotor leftWinchMotor, DcMotor rightWinchMotor)
    {
        this.leftWinchMotor = leftWinchMotor;
        this.rightWinchMotor = rightWinchMotor;


        this.leftWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        this.rightWinchMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leftWinchMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Start()
    {
        leftWinchMotor.setTargetPosition(leftWinchMotor.getCurrentPosition());
        rightWinchMotor.setTargetPosition(rightWinchMotor.getCurrentPosition());

        leftWinchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightWinchMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void Update(double deltaTime)
    {
        System.out.println("targetPosition: " + leftWinchMotor.getTargetPosition());
        System.out.println("currentPosition: " + leftWinchMotor.getCurrentPosition());
        if (shouldUpdatePosition)
        {
            leftWinchMotor.setTargetPosition((int)Math.round(targetPosition));
            rightWinchMotor.setTargetPosition((int)Math.round(targetPosition));
            shouldUpdatePosition = false;
        }

        if (shouldUpdatePower)
        {
            leftWinchMotor.setPower(power);
            rightWinchMotor.setPower(power);
            shouldUpdatePower = false;
        }
    }

    public void Stop()
    {

    }



    public void SetTargetDegrees(double degrees)
    {
        targetPosition = (degrees - Constants.WinchConstants.WINCH_OFFSET) / Constants.WinchConstants.DEGREES_PER_ENCODER_TICK;
        shouldUpdatePosition = true;
    }

    public double GetDegrees()
    {
        return (leftWinchMotor.getCurrentPosition() * Constants.WinchConstants.DEGREES_PER_ENCODER_TICK) + Constants.WinchConstants.WINCH_OFFSET;
    }

    public double GetTargetDegrees()
    {
        return (targetPosition * Constants.WinchConstants.DEGREES_PER_ENCODER_TICK) + Constants.WinchConstants.WINCH_OFFSET;
    }

    public void SetPower(double power)
    {
        this.power = power;
        shouldUpdatePower = true;
    }
    public double GetPower()
    {
        return power;
    }

}
