package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.ColorSensorEx;
import org.firstinspires.ftc.teamcode.core.Timer;

public class Drive_Claw_Manager implements Runnable
{
    private boolean isAlive;

    // Modules
    public ClawModule clawModule;
    public DriveModule driveModule;

    private Timer timer;


    public Drive_Claw_Manager(HardwareMap hardwareMap)
    {
        clawModule = new ClawModule(
                hardwareMap.get(Servo.class, Constants.HardwareConstants.CLAW_SERVO_NAME),
                hardwareMap.get(Servo.class, Constants.HardwareConstants.WRIST_SERVO_NAME),
                hardwareMap.get(Servo.class, Constants.HardwareConstants.LEFT_ARM_SERVO_NAME),
                hardwareMap.get(Servo.class, Constants.HardwareConstants.RIGHT_ARM_SERVO_NAME)/*,
                new ColorSensorEx(hardwareMap.get(ColorSensor.class, Constants.HardwareConstants.TOP_COLOR_SENSOR)),
                new ColorSensorEx(hardwareMap.get(ColorSensor.class, Constants.HardwareConstants.BOTTOM_COLOR_SENSOR))*/);
        driveModule = new DriveModule(
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.FRONT_LEFT_DRIVE_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.FRONT_RIGHT_DRIVE_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.BACK_LEFT_DRIVE_MOTOR_NAME),
                hardwareMap.get(DcMotor.class, Constants.HardwareConstants.BACK_RIGHT_DRIVE_MOTOR_NAME));


        timer = new Timer();
    }


    @Override
    public void run()
    {
        isAlive = true;
        timer.Reset();

        while (isAlive)
        {
            timer.Update();

            clawModule.Update(timer.deltaTime);
            driveModule.Update(timer.deltaTime);
        }
    }



    public void Stop()
    {
        isAlive = false;
    }

}
