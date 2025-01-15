package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Modules.ClawModule;
import org.firstinspires.ftc.teamcode.Modules.DriveModule;
import org.firstinspires.ftc.teamcode.core.Timer;

@TeleOp(name="Basket Auto", group="Robot")
public class BasketAuto extends LinearOpMode {


    private Robot robot;

    @Override
    public void runOpMode() {

        robot = new Robot(hardwareMap);

        waitForStart();

        robot.Start();
        robot.drive_claw_manager.driveModule.SetPathFindingBehavior(DriveModule.PathFindingBehavior.ThreeWheelOdometerPods);
        robot.drive_claw_manager.driveModule.SetPathFindingMotorController(DriveModule.PathFindingMotorController.PID);

        while (opModeIsActive())
        {
            robot.Update();

            robot.drive_claw_manager.driveModule.MoveToPosition(500, 500, 180);
        }

        robot.Stop();
    }

}
