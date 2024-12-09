package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class DriveModule extends Thread {

    private static class Vector3
    {
        private double x;
        private double z;
        private double rotation;
    }

    public enum PathFindingBehavior
    {
        ThreeWheelOdometerPods,
        WheelEncoders,
        None,
    }
    public enum ControlBehavior
    {
        FirstController,
        SecondController,
        Both,
        None,
    }

    // Module
    private final OpMode opMode;
    private boolean stopRequested;

    // Settings
    private PathFindingBehavior pathFindingBehavior;
    private ControlBehavior controlBehavior;
    private double speed;

    // Movement
    private Vector3 inputs;

    private Vector3 movement;
    private boolean isMoving;
    private double targetMovementTime = 0;
    private double startMovementTime = 0;

    private Vector3 targetPosition;
    private boolean isFollowingPath;

    private Vector3 finialMovement;


    public DriveModule(OpMode opMode)
    {
        this.opMode = opMode;
    }



    public void run()
    {
        while (!stopRequested)
        {
            opMode.telemetry.addData("<", "Drive Module Running");

            GetInputs();

            if (inputs.x == 0 && inputs.z == 0 && inputs.rotation == 0)
            {
                if (isFollowingPath)
                {
                    if (pathFindingBehavior == PathFindingBehavior.ThreeWheelOdometerPods)
                    {
                        boolean test1 = true;
                        boolean test2 = false;
                    }
                    else if (pathFindingBehavior == PathFindingBehavior.WheelEncoders)
                    {
                        boolean test1 = true;
                        boolean test2 = false;
                    }
                    else
                    {
                        opMode.telemetry.addData("<", "Can not follow a path if there are not encoder Tracking.");
                    }
                }
                else if (isMoving)
                {
                    if (targetMovementTime == -1)
                    {
                        finialMovement.x = movement.x;
                        finialMovement.z = movement.z;
                        finialMovement.rotation = movement.rotation;
                        isMoving = false;
                    }
                    //        Gets the Elapsed Movement Time
                    else if ((startMovementTime - opMode.getRuntime()) < targetMovementTime)
                    {
                        finialMovement.x = movement.x;
                        finialMovement.z = movement.z;
                        finialMovement.rotation = movement.rotation;
                    }
                    else
                    {
                        ResetMovement();
                    }
                }
            }
            else
            {
                if (isMoving)
                {
                    ResetMovement();
                }
                else if (isFollowingPath)
                {
                    ResetPath();
                }

                finialMovement.x = inputs.x;
                finialMovement.z = inputs.z;
                finialMovement.rotation = inputs.rotation;
            }

            PowerMotors(finialMovement.x, finialMovement.z, finialMovement.rotation);
        }
    }

    public void Stop()
    {
        stopRequested = true;
    }



    public void SetPathFindingBehavior(PathFindingBehavior pathFindingBehavior)
    {
        this.pathFindingBehavior = pathFindingBehavior;
    }
    public void SetControlBehavior(ControlBehavior controlBehavior)
    {
        this.controlBehavior = controlBehavior;
    }
    public void SetSpeed(double speed)
    {
        this.speed = speed;
    }



    public void Move(double x, double z, double rotation)
    {
        movement.x = x;
        movement.z = z;
        movement.rotation = rotation;
        targetMovementTime = -1;
        startMovementTime = -1;
        isMoving = true;

        if (isFollowingPath)
        {
            targetPosition.x = -1;
            targetPosition.z = -1;
            targetPosition.rotation = -1;
            isFollowingPath = false;
        }
    }
    public void MoveForSeconds(double x, double z, double rotation, double seconds)
    {
        movement.x = x;
        movement.z = z;
        movement.rotation = rotation;
        targetMovementTime = seconds;
        startMovementTime = opMode.getRuntime();
        isMoving = true;

        if (isFollowingPath)
        {
            targetPosition.x = -1;
            targetPosition.z = -1;
            targetPosition.rotation = -1;
            isFollowingPath = false;
        }
    }
    public void MoveToPosition(double x, double z, double rotation)
    {
        targetPosition.x = x;
        targetPosition.z = z;
        targetPosition.rotation = rotation;
        isFollowingPath = true;

        if (isMoving)
        {
            movement.x = 0;
            movement.z = 0;
            movement.rotation = 0;
            targetMovementTime = -1;
            startMovementTime = -1;

            isMoving = false;
        }
    }



    private void PowerMotors(double x, double z, double rotation)
    {
        double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);

        HardwareModule.frontLeftMotor.setPower(((z + x + rotation) / denominator) * speed);
        HardwareModule.frontRightMotor.setPower(((z - x - rotation) / denominator) * speed);
        HardwareModule.backLeftMotor.setPower(((z - x + rotation) / denominator) * speed);
        HardwareModule.backRightMotor.setPower(((z + x - rotation) / denominator) * speed);
    }

    private void GetInputs()
    {
        if (controlBehavior == ControlBehavior.FirstController)
        {
            inputs.x = opMode.gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            inputs.z = -opMode.gamepad1.left_stick_y; // Remember, Y stick is reversed!
            inputs.rotation = opMode.gamepad1.right_stick_x;
        }
        else if (controlBehavior == ControlBehavior.SecondController)
        {
            inputs.x = opMode.gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            inputs.z = -opMode.gamepad2.left_stick_y; // Remember, Y stick is reversed!
            inputs.rotation = opMode.gamepad2.right_stick_x;
        }
        else if (controlBehavior == ControlBehavior.Both)
        {
            inputs.x = Clamp(opMode.gamepad1.left_stick_x + opMode.gamepad2.left_stick_y) * 1.1; // Counteract imperfect strafing
            inputs.z = -Clamp(opMode.gamepad1.left_stick_y + opMode.gamepad2.left_stick_y); // Remember, Y stick is reversed!
            inputs.rotation = Clamp(opMode.gamepad1.right_stick_x + opMode.gamepad2.right_stick_x);
        }
    }

    private double Clamp(double value)
    {
        if (value > 1)
        {
            value = 1;
        }
        else if (value < -1)
        {
            value = -1;
        }
        return  value;
    }


    private void ResetMovement()
    {
        movement.x = 0;
        movement.z = 0;
        movement.rotation = 0;
        targetMovementTime = -1;
        startMovementTime = -1;
    }
    private void ResetPath()
    {
        targetPosition.x = -1;
        targetPosition.z = -1;
        targetPosition.rotation = -1;
        isFollowingPath = false;
    }



}
