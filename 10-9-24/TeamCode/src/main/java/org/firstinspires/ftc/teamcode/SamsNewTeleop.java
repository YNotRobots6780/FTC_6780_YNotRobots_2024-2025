package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Modules.ClawModule;

@TeleOp(name="Sam's New 6780 Code", group="Robot")
public class SamsNewTeleop extends LinearOpMode {


    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.Start();

        while (opModeIsActive())
        {
            robot.Update();


            robot.drive_claw_manager.driveModule.Move(-gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x * 1.25);

           //Toggle
            if (gamepad1.a)
            {
                robot.drive_claw_manager.clawModule.OpenClaw();
            }
            else if (gamepad1.b)
            {
                robot.drive_claw_manager.clawModule.CloseClaw();
            }

            robot.drive_claw_manager.clawModule.SetWristDegrees(gamepad1.left_trigger * 90);
/*
            if ()
            {
                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Up);
            }
            else if ()
            {
                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Out);
            }
            else if ()
            {
                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.HalfDown);
            }
            else if ()
            {
                robot.drive_claw_manager.clawModule.SetArmPosition(ClawModule.ArmPosition.Down);
            }
*/







            robot.winch_elevator_manager.winchModule.SetTargetDegrees((gamepad1.right_trigger * 58.5) - 3.5);
            robot.winch_elevator_manager.winchModule.SetPower(Constants.WINCH_SPEED);

        }

        robot.Stop();
    }

}
