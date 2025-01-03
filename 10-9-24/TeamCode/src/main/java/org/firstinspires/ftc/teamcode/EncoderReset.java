package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;

@TeleOp(name="RESET ENCODERS", group="Robot")
public class EncoderReset extends LinearOpMode {


    @Override
    public void runOpMode()
    {

        // Drive Motors
        DcMotor frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // EX: 0
        DcMotor frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 3
        DcMotor backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // EX: 1
        DcMotor backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); // 2

        DcMotor elevatorMotorRight = hardwareMap.get(DcMotor.class,"elevatorMotorRight"); // 0
        DcMotor elevatorMotorLeft = hardwareMap.get(DcMotor.class,"elevatorMotorLeft"); // EX: 3

        DcMotor winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight"); // 1
        DcMotor winchMotorLeft =  hardwareMap.get(DcMotor.class,"winchMotorLeft"); // EX: 2


        frontLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        winchMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    }



}
