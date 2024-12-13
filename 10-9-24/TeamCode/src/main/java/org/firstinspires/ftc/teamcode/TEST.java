package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@TeleOp(name="6780 Test", group="Robot")

public class TEST extends OpMode {

    public static DcMotor elevatorMotorRight;
    public static DcMotor elevatorMotorLeft;
    public static DcMotor winchMotorRight;
    public static DcMotor winchMotorLeft;

    @Override
    public void init() {

        elevatorMotorRight = hardwareMap.get(DcMotor.class,"elevatorMotorRight"); // 0
        elevatorMotorLeft = hardwareMap.get(DcMotor.class,"elevatorMotorLeft"); // EX: 3

        winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight"); // 1
        winchMotorLeft =  hardwareMap.get(DcMotor.class,"winchMotorLeft"); // EX: 2


        elevatorMotorRight.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        winchMotorRight.setDirection(DcMotor.Direction.FORWARD);
        winchMotorLeft.setDirection(DcMotor.Direction.REVERSE);


        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        winchMotorLeft.setTargetPosition(120);
        winchMotorRight.setTargetPosition(120);
        winchMotorLeft.setPower(1);
        winchMotorRight.setPower(1);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);


        elevatorMotorLeft.setTargetPosition(0);
        elevatorMotorRight.setTargetPosition(0);
        elevatorMotorLeft.setPower(1);
        elevatorMotorRight.setPower(1);
        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }

    @Override
    public void loop() {

        telemetry.addData("<", "Elevator: " + elevatorMotorLeft.getCurrentPosition());
        telemetry.addData("<", "Elevator Right: " + elevatorMotorRight.getCurrentPosition());
        telemetry.addData("<", "Winch: " + winchMotorLeft.getCurrentPosition());
        telemetry.addData("<", "Winch Right: " + winchMotorRight.getCurrentPosition());


        if (gamepad1.left_trigger > 0.25)
        {
            elevatorMotorLeft.setTargetPosition(elevatorMotorLeft.getCurrentPosition() + 100);
            elevatorMotorRight.setTargetPosition(elevatorMotorLeft.getCurrentPosition() + 100);
        }
        else if (gamepad1.left_bumper)
        {
            elevatorMotorLeft.setTargetPosition(elevatorMotorLeft.getCurrentPosition() + 100);
            elevatorMotorRight.setTargetPosition(elevatorMotorLeft.getCurrentPosition() + 100);
            elevatorMotorLeft.setPower(1);
            elevatorMotorRight.setPower(1);
            elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }

        winchMotorLeft.setTargetPosition(120 + (elevatorMotorLeft.getCurrentPosition() / 50));
        winchMotorRight.setTargetPosition(120 + (elevatorMotorRight.getCurrentPosition() / 50));

    }
}
