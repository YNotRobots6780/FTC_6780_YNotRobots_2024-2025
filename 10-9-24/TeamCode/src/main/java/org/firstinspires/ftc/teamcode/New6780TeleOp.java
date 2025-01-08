package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.ClawModule;
import org.firstinspires.ftc.teamcode.core.Timer;

@TeleOp(name="New 6780 Teleop", group="Robot")
public class New6780TeleOp extends LinearOpMode {

    public enum Position
    {
        Pickup,
        HighBasket,
        LowBasket,
        HighChamber,
        LowChamber
    }

    private Robot robot;

    private Position position;

    private boolean isCurrentlySwitching;

    // Claw
    private boolean isClawOpen;
    private boolean isClawPressed;

    private boolean isClawPickingUp;
    private boolean isClawPickingUpPressed;
    private Timer pickingUpTimer;

    // Basket
    private boolean isScoringHighBasket = true;
    private boolean isBasketPressed;

    // Chamber
    private boolean isScoringHighChamber = true;
    private boolean isChamberPressed;



    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        waitForStart();

        robot.Start();

        position = Position.Pickup;
        pickingUpTimer = new Timer();
        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);
        robot.drive_claw_manager.clawModule.SetWristDegrees(0);


        while (opModeIsActive())
        {
            robot.Update();

            pickingUpTimer.Update();

            NormalDrive();
        }

        robot.Stop();
    }

    public void NormalDrive()
    {
        robot.drive_claw_manager.driveModule.Move(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x * 1.25);

        robot.drive_claw_manager.clawModule.SetWristDegrees(gamepad1.left_trigger * 90);

        if (gamepad1.x)
        {
            if (!isClawPressed)
            {
                isClawPressed = true;
                isClawOpen = !isClawOpen;
                if (isClawOpen)
                {
                    robot.drive_claw_manager.clawModule.OpenClaw();
                }
                else
                {
                    robot.drive_claw_manager.clawModule.CloseClaw();
                }
            }
        }
        else
        {
            isClawPressed = false;
        }


        if (gamepad1.y)
        {
            robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);
        }
        else if (gamepad1.b)
        {
            robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Out);
        }
        else if (gamepad1.a)
        {
            if (position == Position.Pickup)
            {
                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Down);
            }
            else
            {
                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.HalfDown);
            }
        }

        if (gamepad1.dpad_down)
        {
            position = Position.Pickup;
            isCurrentlySwitching = true;
        }
        else if (gamepad1.dpad_left)
        {
            // Baskets
            if (!isBasketPressed)
            {
                isCurrentlySwitching = true;
                isBasketPressed = true;
                isScoringHighBasket = !isScoringHighBasket;
                if (isScoringHighBasket)
                {
                    position = Position.HighBasket;
                }
                else
                {
                    position = Position.LowBasket;
                }
            }
            else
            {
                isBasketPressed = false;
            }
        }
        else if (gamepad1.dpad_right)
        {
            // Chambers
            if (!isChamberPressed)
            {
                isCurrentlySwitching = true;
                isChamberPressed = true;
                isScoringHighChamber = !isScoringHighChamber;
                if (isScoringHighChamber)
                {
                    position = Position.HighChamber;
                }
                else
                {
                    position = Position.LowChamber;
                }
            }
            else
            {
                isChamberPressed = false;
            }
        }

        switch (position)
        {
            case Pickup:
            {
                if (isCurrentlySwitching && (robot.winch_elevator_manager.elevatorModule.GetPosition() > 100 || robot.winch_elevator_manager.winchModule.GetDegrees() > 20))
                {
                    robot.winch_elevator_manager.MoveWinch_And_Elevator(15, 50);
                    break;
                }
                else
                {
                    isCurrentlySwitching = false;
                }

                if (gamepad1.right_trigger > 0.25)
                {
                    robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.winch_elevator_manager.elevatorModule.SetPower(gamepad1.right_trigger);
                    isClawPickingUp = false;
                }
                else if (gamepad1.right_bumper)
                {
                    robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    robot.winch_elevator_manager.elevatorModule.SetPower(-1);
                    isClawPickingUp = false;
                }
                else
                {
                    robot.winch_elevator_manager.elevatorModule.SetPower(0);
                }

                if (gamepad1.x)
                {
                    if (!isClawPickingUpPressed)
                    {
                        isClawPickingUpPressed = true;
                        isClawPickingUp = !isClawPickingUp;
                        pickingUpTimer.Reset();
                    }
                }
                else
                {
                    isClawPickingUpPressed = false;
                }

                if (isClawPickingUp)
                {
                    if (pickingUpTimer.timeSinceStart < 0.25)
                    {
                        isClawOpen = true;
                        robot.drive_claw_manager.clawModule.OpenClaw();
                    }
                    if (pickingUpTimer.timeSinceStart < 1.25)
                    {
                        robot.winch_elevator_manager.winchModule.SetTargetDegrees(4);
                    }
                    else if (pickingUpTimer.timeSinceStart < 1.5)
                    {
                        isClawOpen = false;
                        robot.drive_claw_manager.clawModule.CloseClaw();
                    }
                    else if (pickingUpTimer.timeSinceStart > 1.5)
                    {
                        robot.winch_elevator_manager.winchModule.SetTargetDegrees(15);

                        robot.winch_elevator_manager.elevatorModule.SetPosition(15);
                    }
                }
                else
                {
                    robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Down);
                    robot.winch_elevator_manager.winchModule.SetTargetDegrees(15);
                }

                break;
            }
            case HighBasket:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(43, 1000);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);
                break;
            }
            case LowBasket:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(40, 350);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);
                break;
            }
            case HighChamber:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(40, 200);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);
                break;
            }
            case  LowChamber:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(10, 50);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);
                break;
            }
        }
    }

}
