package org.firstinspires.ftc.teamcode.core;

import com.qualcomm.robotcore.hardware.DcMotor;

public class Encoder {

    private DcMotor motor;

    public Encoder(DcMotor encoderMotor)
    {
        motor = encoderMotor;

        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }


    // Test
    public int getCurrentTicks()
    {
        return motor.getCurrentPosition();
    }

    public int getTargetTicks()
    {
        return motor.getTargetPosition();
    }

    public void  setTargetTicks(int ticks)
    {
        motor.setTargetPosition(ticks);
    }

}
