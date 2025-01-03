package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.core.ColorSensorEx;

public class ClawModule
{
    public enum ArmPosition
    {
        Up,
        Out,
        HalfDown,
        Down,
    }

    // Module
    private final Servo wristServo;
    private final Servo leftArmServo;
    private final Servo rightArmServo;
    private final Servo clawServo;

    // private final ColorSensorEx topColorSensor;
    // private final ColorSensorEx bottomColorSensor;

    // Claw
    private boolean isClawOpen;
    private boolean isClawMoving;

    // Wrist
    private double wristDegrees;
    private boolean isWristArm = false;

    // Arm
    private ArmPosition targetArmPosition;
    private boolean isMovingArm = false;



    public ClawModule(Servo clawServo, Servo wristServo, Servo leftArmServo, Servo rightArmServo/*, ColorSensorEx topColorSensor, ColorSensorEx bottomColorSensor*/)
    {
        this.clawServo = clawServo;
        this.wristServo = wristServo;
        this.leftArmServo = leftArmServo;
        this.rightArmServo = rightArmServo;
        this.rightArmServo.setDirection(Servo.Direction.REVERSE);

        // this.topColorSensor = topColorSensor;
        // this.bottomColorSensor = bottomColorSensor;
    }

    public void Update(double deltaTime)
    {
        if (isWristArm)
        {
            isWristArm = false;
            wristServo.setPosition((Constants.ClawConstants.WRIST_DEFAULT_POSITION_DEGREES + wristDegrees) / Constants.ClawConstants.WRIST_SERVO_ROTATION_AMOUNT);
        }

        if (isMovingArm)
        {
            isMovingArm = false;
            double position = 0;
            switch (targetArmPosition)
            {
                case Up:
                    position = Constants.ClawConstants.ARM_STRAIGHT_UP_POSITION_DEGREES / Constants.ClawConstants.ARM_SERVO_ROTATION_AMOUNT;
                    break;
                case Out:
                    position = Constants.ClawConstants.ARM_STRAIGHT_OUT_POSITION_DEGREES / Constants.ClawConstants.ARM_SERVO_ROTATION_AMOUNT;
                    break;
                case HalfDown:
                    position = Constants.ClawConstants.ARM_HALF_DOWN_POSITION_DEGREES / Constants.ClawConstants.ARM_SERVO_ROTATION_AMOUNT;
                    break;
                case Down:
                    position = Constants.ClawConstants.ARM_STRAIGHT_DOWN_POSITION_DEGREES / Constants.ClawConstants.ARM_SERVO_ROTATION_AMOUNT;
                    break;
            }
            leftArmServo.setPosition(position);
            rightArmServo.setPosition(position);
        }

        if (isClawMoving)
        {
            clawServo.setPosition((isClawOpen ? Constants.ClawConstants.CLAW_OPEN_DEGREES : Constants.ClawConstants.CLAW_CLOSE_DEGREES)
                    / Constants.ClawConstants.CLAW_SERVO_ROTATION_AMOUNT);
            isClawMoving = false;
        }
    }

    public void SetWristDegrees(double degrees)
    {
        wristDegrees = degrees;
        isWristArm = true;
    }


    public void CloseClaw()
    {
        if (isClawOpen)
        {
            isClawMoving = true;
            isClawOpen = false;
        }
    }

    public void OpenClaw()
    {
        if (!isClawOpen)
        {
            isClawMoving = true;
            isClawOpen = true;
        }
    }

    public void SetArmPosition(ArmPosition armPosition)
    {
        this.targetArmPosition = armPosition;
        isMovingArm = true;
    }

}
