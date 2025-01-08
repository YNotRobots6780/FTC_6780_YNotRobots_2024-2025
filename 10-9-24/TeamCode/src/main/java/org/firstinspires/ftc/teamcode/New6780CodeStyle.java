package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.Modules.ClawModule;

@TeleOp(name="New 6780 Code Style", group="Robot")
public class New6780CodeStyle extends LinearOpMode {


    @Override
    public void runOpMode() {
        // Robot robot = new Robot(hardwareMap);

        DcMotor leftElevatorMotor = hardwareMap.get(DcMotor.class, Constants.HardwareConstants.LEFT_ELEVATOR_MOTOR_NAME);

        waitForStart();

        // robot.Start();

        while (opModeIsActive())
        {
            // robot.Update();


            System.out.println((leftElevatorMotor.getCurrentPosition() * Constants.ElevatorConstants.DISTANCE_PER_ENCODER_TICK) + Constants.ElevatorConstants.EXTENSION_OFFSET);

        }

        // robot.Stop();
    }

}
