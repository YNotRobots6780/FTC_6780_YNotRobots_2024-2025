package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.MathF;
import org.firstinspires.ftc.teamcode.core.Timer;

public class ClawModule extends Thread
{
    // Module
    private final OpMode opMode;
    private boolean stopRequested;
    private Timer timer;

    // Wrist
    private boolean isWristRotationContinuous = false;
    private double wristMovement = 0;

    // Arm
    private double targetArmDegrees = 0;


    public ClawModule(OpMode opMode)
    {
        this.opMode = opMode;
    }




    public void run()
    {
        while (!stopRequested)
        {
            timer.Update();
            opMode.telemetry.addData("<", "ClawModuleRunning");

            if (isWristRotationContinuous)
            {
                // wristMovement Is Power
                HardwareModule.wristServo.setPosition(MathF.Clamp(HardwareModule.wristServo.getPosition() +
                        ((timer.deltaTime * Constants.ClawConstants.WRIST_DEGREES_PER_SECOND * MathF.Clamp(wristMovement, -1, 1))
                        / Constants.ClawConstants.WRIST_SERVO_ROTATION_AMOUNT), 0, 1));
            }
            else
            {
                // wristMovement is Degrees
                HardwareModule.wristServo.setPosition(MathF.Clamp(wristMovement / Constants.ClawConstants.WRIST_SERVO_ROTATION_AMOUNT, 0, 1));
            }

            HardwareModule.wristServo.setPosition(MathF.Clamp(targetArmDegrees / Constants.ClawConstants.ARM_SERVO_ROTATION_AMOUNT, 0, 1));

        }
    }

    public void Stop()
    {
        stopRequested = true;
    }


    // Settings
    public void SetWristRotationMode(boolean isContinuous)
    {
        isWristRotationContinuous = isContinuous;
    }



    // Commands
    public void CloseClaw()
    {
        HardwareModule.clawServo.setPosition(Constants.ClawConstants.CLAW_CLOSE_DEGREES / Constants.ClawConstants.CLAW_SERVO_ROTATION_AMOUNT);
    }

    public void OpenClaw()
    {
        HardwareModule.clawServo.setPosition(Constants.ClawConstants.CLAW_OPEN_DEGREES / Constants.ClawConstants.CLAW_SERVO_ROTATION_AMOUNT);
    }

    public void SetWristPosition(double degrees_power)
    {
        wristMovement = degrees_power;
    }

    public void SetArmPosition(double targetArmDegrees)
    {
        this.targetArmDegrees = targetArmDegrees;
    }



}
