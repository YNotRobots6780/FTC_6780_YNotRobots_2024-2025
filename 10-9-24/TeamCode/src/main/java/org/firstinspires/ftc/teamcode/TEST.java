package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="6780 Test", group="Robot")

public class TEST extends LinearOpMode {


    @Override
    public void runOpMode() {
        hardwareMap.get(DcMotor.class, "front_right");

        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.Start();

        while (opModeIsActive())
        {


            robot.Update();
        }

        robot.Stop();
    }

}
