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
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // EX: 0
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 3
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // EX: 1
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); // 2


        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);


        Robot robot = new Robot(hardwareMap);

        waitForStart();

        robot.Start();

        while (opModeIsActive())
        {
            double x = -gamepad1.left_stick_x;
            double z = gamepad1.left_stick_y * 1.1;
            double rotation = -gamepad1.right_stick_x;

            double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);

            frontLeftMotor.setPower(((z + x + rotation) / denominator));
            frontRightMotor.setPower(((z - x - rotation) / denominator));
            backLeftMotor.setPower(((z - x + rotation) / denominator));
            backRightMotor.setPower(((z + x - rotation) / denominator));

            robot.Update();
        }

        robot.Stop();
    }

    private void PowerDriveMotors(double x, double z, double rotation, double speed)
    {
    }

}
