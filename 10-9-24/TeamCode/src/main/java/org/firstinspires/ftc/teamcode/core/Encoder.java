package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {

    private final DcMotor motor;
    private int encoderModifier = 1;

    public Encoder(DcMotor encoderMotor)
    {
        motor = encoderMotor;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // Test
    public int getCurrentTicks()
    {
        return motor.getCurrentPosition() * encoderModifier;
    }

    public int getTargetTicks()
    {
        return motor.getTargetPosition() * encoderModifier;
    }

    public void setTargetTicks(int ticks)
    {
        motor.setTargetPosition(ticks * encoderModifier);
    }

    public void SetMode(DcMotor.RunMode runMode)
    {
        motor.setMode(runMode);
    }

    public void SetReverse(boolean isReverse)
    {
        encoderModifier = (isReverse ? -1 : 1);
    }

}
