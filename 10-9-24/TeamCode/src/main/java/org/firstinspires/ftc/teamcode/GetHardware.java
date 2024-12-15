package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;

@TeleOp(name="6780 Get Hardware", group="Robot")

public class GetHardware extends OpMode {
    @Override
    public void init() {
        GetFromHardwareMap();

        Configure();
    }

    @Override
    public void loop() {

    }



    private void GetFromHardwareMap()
    {
        // Drive Motors
        HardwareModule.frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // EX: 0
        HardwareModule.frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 3
        HardwareModule.backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // EX: 1
        HardwareModule.backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); // 2

        HardwareModule.elevatorMotorRight = hardwareMap.get(DcMotor.class,"elevatorMotorRight");  // 0
        HardwareModule.elevatorMotorLeft = hardwareMap.get(DcMotor.class,"elevatorMotorLeft"); // EX: 3

        HardwareModule.winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight"); // 1
        HardwareModule.winchMotorLeft =  hardwareMap.get(DcMotor.class,"winchMotorLeft"); // EX: 2

        // HardwareModule.armServoRight = hardwareMap.get(Servo.class,"armServoRight");
        // HardwareModule.armServoLeft = hardwareMap.get(Servo.class,"armServoLeft");
        // HardwareModule.clawServo = hardwareMap.get(Servo.class,"clawServo");
        // HardwareModule.wristServo = hardwareMap.get(Servo.class,"wristServo");

        HardwareModule.leftLight = hardwareMap.get(Servo.class,"leftLight"); // 4, or 5
        HardwareModule.rightLight = hardwareMap.get(Servo.class,"rightLight"); // 4, or 5

        // Custom Encoder Module
        HardwareModule.leftOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        HardwareModule.rightOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_right"));
        HardwareModule.backOdometer = new Encoder(hardwareMap.get(DcMotor.class, "back_left"));
    }


    private static void Configure()
    {
        // Drive Motors
        HardwareModule.frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        HardwareModule.frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        HardwareModule.backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        HardwareModule.backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        HardwareModule.elevatorMotorRight.setDirection(DcMotor.Direction.FORWARD);
        HardwareModule.elevatorMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        HardwareModule.winchMotorRight.setDirection(DcMotor.Direction.REVERSE);
        HardwareModule.winchMotorLeft.setDirection(DcMotor.Direction.FORWARD);


        // HardwareModule.armServoLeft.setDirection(Servo.Direction.REVERSE);
    }
}
