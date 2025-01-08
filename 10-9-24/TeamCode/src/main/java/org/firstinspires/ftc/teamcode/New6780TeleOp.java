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

    private boolean isScoring;
    private boolean isScoringPressed;
    private Timer scoringTimer;



    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);

        waitForStart();

        robot.Start();

        position = Position.Pickup;
        pickingUpTimer = new Timer();
        scoringTimer = new Timer();
        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);
        robot.drive_claw_manager.clawModule.SetWristDegrees(0);


        while (opModeIsActive())
        {
            robot.Update();

            pickingUpTimer.Update();
            scoringTimer.Update();

            NormalDrive();
        }

        robot.Stop();
    }

    public void NormalDrive()
    {
        robot.drive_claw_manager.driveModule.Move(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x * 1.25);

        robot.drive_claw_manager.clawModule.SetWristDegrees(gamepad1.left_trigger * 90);


        if (gamepad1.dpad_down)
        {
            position = Position.Pickup;
            isCurrentlySwitching = true;
            isScoring = false;
        }
        else if (gamepad1.dpad_left)
        {
            // Baskets
            if (!isBasketPressed)
            {
                isCurrentlySwitching = true;
                isScoring = false;
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
        }
        else if (gamepad1.dpad_right)
        {
            // Chambers
            if (!isChamberPressed)
            {
                isCurrentlySwitching = true;
                isScoring = false;
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
        }
        else
        {
            isBasketPressed = false;
            isChamberPressed = false;
        }

        switch (position)
        {
            case Pickup:
            {
                if (isCurrentlySwitching)
                {
                    if (robot.winch_elevator_manager.elevatorModule.GetPosition() > 100 || robot.winch_elevator_manager.winchModule.GetDegrees() > 20)
                    {
                        robot.winch_elevator_manager.MoveWinch_And_Elevator(15, 50);
                        break;
                    }
                    else
                    {
                        isCurrentlySwitching = false;
                    }
                }
                else
                {
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
                }
                break;
            }
            case HighBasket:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(43, 860);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);

                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);

                if (gamepad1.right_trigger > 0.25)
                {
                    if (!isChamberPressed) {
                        isChamberPressed = true;
                        isScoring = !isScoring;
                        scoringTimer.Reset();
                    }
                }
                else
                {
                    isChamberPressed = false;
                }

                if (isScoring)
                {
                    if (scoringTimer.timeSinceStart < 1)
                    {
                        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Out);
                    }
                    else if (scoringTimer.timeSinceStart < 1.5)
                    {
                        robot.drive_claw_manager.clawModule.OpenClaw();
                    }
                    else
                    {
                        position = Position.Pickup;
                        isCurrentlySwitching = true;
                        isScoring = false;
                    }
                }

                break;
            }
            case LowBasket:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(40, 350);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);

                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);

                if (gamepad1.right_trigger > 0.25)
                {
                    if (!isChamberPressed) {
                        isChamberPressed = true;
                        isScoring = !isScoring;
                        scoringTimer.Reset();
                    }
                }
                else
                {
                    isChamberPressed = false;
                }

                if (isScoring)
                {
                    if (scoringTimer.timeSinceStart < 1)
                    {
                        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Out);
                    }
                    else if (scoringTimer.timeSinceStart < 1.5)
                    {
                        robot.drive_claw_manager.clawModule.OpenClaw();
                    }
                    else
                    {
                        position = Position.Pickup;
                        isCurrentlySwitching = true;
                        isScoring = false;
                    }
                }

                break;
            }
            case HighChamber:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(40, 200);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);


                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);

                if (gamepad1.right_trigger > 0.25)
                {
                    if (!isChamberPressed) {
                        isChamberPressed = true;
                        isScoring = !isScoring;
                        scoringTimer.Reset();
                    }
                }
                else
                {
                    isChamberPressed = false;
                }

                if (isScoring)
                {
                    if (scoringTimer.timeSinceStart < 1)
                    {
                        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Out);
                    }
                    else if (scoringTimer.timeSinceStart < 1.5)
                    {
                        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Down);
                    }
                    else if (scoringTimer.timeSinceStart < 2)
                    {
                        robot.drive_claw_manager.clawModule.OpenClaw();
                    }
                    else
                    {
                        position = Position.Pickup;
                        isCurrentlySwitching = true;
                        isScoring = false;
                    }
                }


                break;
            }
            case  LowChamber:
            {
                robot.winch_elevator_manager.MoveWinch_And_Elevator(10, 50);
                robot.winch_elevator_manager.elevatorModule.SetMode(DcMotor.RunMode.RUN_TO_POSITION);
                robot.winch_elevator_manager.elevatorModule.SetPower(1);


                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);

                if (gamepad1.right_trigger > 0.25)
                {
                    if (!isChamberPressed) {
                        isChamberPressed = true;
                        isScoring = !isScoring;
                        scoringTimer.Reset();
                    }
                }
                else
                {
                    isChamberPressed = false;
                }

                if (isScoring)
                {
                    if (scoringTimer.timeSinceStart < 1)
                    {
                        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Out);
                    }
                    else if (scoringTimer.timeSinceStart < 1.5)
                    {
                        robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Down);
                    }
                    else if (scoringTimer.timeSinceStart < 2)
                    {
                        robot.drive_claw_manager.clawModule.OpenClaw();
                    }
                    else
                    {
                        position = Position.Pickup;
                        isCurrentlySwitching = true;
                        isScoring = false;
                    }
                }
                
                break;
            }
        }
    }

}
