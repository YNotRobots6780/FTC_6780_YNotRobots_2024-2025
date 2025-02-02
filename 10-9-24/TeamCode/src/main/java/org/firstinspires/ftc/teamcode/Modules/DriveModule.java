package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.ftccommon.internal.manualcontrol.parameters.ImuParameters;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.PIDController;

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

        public String ToString()
        {
            return x + ", " + z + ", " + rotation;
        }
    }

    public enum DriveMode
    {
        RobotCentric,
        FieldCentric
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
    private IMU imu;

    // Settings
    private DriveMode driveMode = DriveMode.RobotCentric;
    private PathFindingBehavior pathFindingBehavior;
    private PathFindingMotorController pathFindingMotorController;
    private double speed = 1;
    private PathfindingModule pathfindingModule;

    // Movement
    private Vector3 movement;
    private boolean isMoving;
    private double targetMovementTime = 0;
    private double movementTime = 0;
    private double orientation;
    private double startOrientation;
    private PIDController turningPIDController;
    private PIDController xPIDController;
    private PIDController zPIDController;


    private Vector3 targetPosition;
    private boolean isFollowingPath;

    private Vector3 finialMovement;

    // TESTING!!!!

    public DriveModule(DcMotor frontLeftMotor, DcMotor frontRightMotor, DcMotor backLeftMotor, DcMotor backRightMotor, IMU imu, PathfindingModule pathfindingModule)
    {
        this.frontLeftMotor = frontLeftMotor;
        this.frontRightMotor = frontRightMotor;
        this.backLeftMotor = backLeftMotor;
        this.backRightMotor = backRightMotor;
        this.imu = imu;
        this.pathfindingModule = pathfindingModule;

        this.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        this.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        this.backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        IMU.Parameters imuParameters = new IMU.Parameters(Constants.IMUConstants.ROBOT_ORIENTATION);
        imu.initialize(imuParameters);


        movement = new Vector3();
        targetPosition = new Vector3();
        finialMovement = new Vector3();

        turningPIDController = new PIDController(Constants.DriveConstants.TURNING_KP, Constants.DriveConstants.TURNING_MAX_ERROR,
                Constants.DriveConstants.TURNING_KI, Constants.DriveConstants.TURNING_KI_ACTIVE_ZONE, Constants.DriveConstants.TURNING_KD);

        xPIDController = new PIDController(Constants.DriveConstants.X_KP, Constants.DriveConstants.X_MAX_ERROR,
                Constants.DriveConstants.X_KI, Constants.DriveConstants.X_KI_ACTIVE_ZONE, Constants.DriveConstants.X_KD);

        zPIDController = new PIDController(Constants.DriveConstants.Z_KP, Constants.DriveConstants.Z_MAX_ERROR,
                Constants.DriveConstants.Z_KI, Constants.DriveConstants.Z_KI_ACTIVE_ZONE, Constants.DriveConstants.Z_KD);
    }


    public void Start()
    {
        startOrientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }
    public void Update(double deltaTime)
    {
        if (driveMode == DriveMode.FieldCentric)
        {
            orientation = imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES) - startOrientation;
        }

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
                    finialMovement.x = xPIDController.Calculate(pathfindingModule.x, targetPosition.x, deltaTime);
                    finialMovement.z = xPIDController.Calculate(pathfindingModule.z, targetPosition.z, deltaTime);
                    finialMovement.rotation = xPIDController.Calculate(pathfindingModule.rotation, targetPosition.rotation, deltaTime);
                    break;
                case LinearInterpretation:
                    System.out.println("LinearInterpretation Has Not been Implemented, Please use one of the other Options");
                    break;
            }
        }

        if (driveMode == DriveMode.FieldCentric)
        {
            Vector3 oldMovement = finialMovement;

            double orientationRadians = Math.toRadians(orientation);

            finialMovement.x = (oldMovement.x * Math.cos(orientationRadians)) - (oldMovement.z * Math.sin(orientationRadians));
            finialMovement.z = (oldMovement.x * Math.sin(orientationRadians)) + (oldMovement.z * Math.cos(orientationRadians));
            finialMovement.rotation = turningPIDController.Calculate(orientation, oldMovement.rotation, deltaTime);

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
    public void SetDriveMode(DriveMode driveMode)
    {
        this.driveMode = driveMode;
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
    public void Move(double x, double z, double rotationX, double rotationY)
    {
        movement.x = x;
        movement.z = z;
        movement.rotation = AngleFromVector(rotationX, rotationY);
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


    private static double AngleFromVector(double x, double y)
    {
        double angle = 0;
        angle = Math.acos(y / (Math.sqrt(1) * Math.sqrt((x * x) + (y * y)))) * (180 / Math.PI);
        if (x < 0)
            return -angle;
        return angle;
    }

}
