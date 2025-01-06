package org.firstinspires.ftc.teamcode.Modules;

import org.firstinspires.ftc.teamcode.core.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

public class DriveModule {

    private static class Vector3
    {
        private double x;
        private double z;
        private double rotation;

        public Vector3()
        {
            x = 0;
            z = 0;
            rotation = 0;
        }
        public Vector3(double x, double z, double rotation)
        {
            this.x = x;
            this.z = z;
            this.rotation = rotation;
        }

        public boolean IsEqual(Vector3 targetVector)
        {
            return targetVector.x == x && targetVector.z == z && targetVector.rotation == rotation;
        }
    }

    public enum PathFindingBehavior
    {
        ThreeWheelOdometerPods,
        WheelEncoders,
        None,
    }

    public enum PathFindingMotorController
    {
        BangBang,
        PID,
        LinearInterpretation,
    }


    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;

    // Settings
    private PathFindingBehavior pathFindingBehavior;
    private PathFindingMotorController pathFindingMotorController;
    private double speed = 1;

    // Movement
    private Vector3 movement;
    private boolean isMoving;
    private double targetMovementTime = 0;
    private double movementTime = 0;


    private Vector3 targetPosition;
    private boolean isFollowingPath;

    private Vector3 finialMovement;


    public DriveModule(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor)
    {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;


        this.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        movement = new Vector3();
        targetPosition = new Vector3();
        finialMovement = new Vector3();
    }


    public void Start()
    {

    }
    public void Update(double deltaTime)
    {
        if (isMoving)
        {
            if (targetMovementTime > 0)
            {
                // Timed Movement
                if (movementTime == 0)
                {
                    // First Frame
                    finialMovement = movement;
                }
                movementTime += deltaTime;

                if (movementTime > targetMovementTime)
                {
                    finialMovement = new Vector3();
                    ResetMovement();
                }
            }
            else
            {
                finialMovement = movement;
            }
        }
        else if (isFollowingPath)
        {
            switch (pathFindingMotorController)
            {
                case BangBang:
                    // if ()
                    break;
                case PID:
                    System.out.println("PID Has Not been Implemented, Please use one of the other Options");
                    break;
                case LinearInterpretation:
                    System.out.println("LinearInterpretation Has Not been Implemented, Please use one of the other Options");
                    break;
            }
        }

        PowerMotors(finialMovement.x, finialMovement.z, finialMovement.rotation);
    }
    public void Stop()
    {

    }


    public void SetPathFindingMotorController(PathFindingMotorController pathFindingMotorController)
    {
        this.pathFindingMotorController = pathFindingMotorController;
    }
    public void SetPathFindingBehavior(PathFindingBehavior pathFindingBehavior)
    {
        this.pathFindingBehavior = pathFindingBehavior;
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
        isMoving = true;

        if (isFollowingPath)
        {
            ResetPath();
        }
    }

    public void MoveForSeconds(double x, double z, double rotation, double seconds)
    {
        movement.x = x;
        movement.z = z;
        movement.rotation = rotation;
        targetMovementTime = seconds;
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
            ResetMovement();
        }
    }



    private void PowerMotors(double x, double z, double rotation)
    {
        double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);

        frontLeftMotor.setPower(((z + x + rotation) / denominator) * speed);
        frontRightMotor.setPower(((z - x - rotation) / denominator) * speed);
        backLeftMotor.setPower(((z - x + rotation) / denominator) * speed);
        backRightMotor.setPower(((z + x - rotation) / denominator) * speed);
    }

    private void ResetMovement()
    {
        movement.x = 0;
        movement.z = 0;
        movement.rotation = 0;
        targetMovementTime = -1;
    }
    private void ResetPath()
    {
        targetPosition.x = -1;
        targetPosition.z = -1;
        targetPosition.rotation = -1;
        isFollowingPath = false;
    }



}
