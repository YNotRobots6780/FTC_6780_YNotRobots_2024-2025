package org.firstinspires.ftc.teamcode.Modules;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;

public class HardwareModule {

    /* Declare OpMode members. */

    public static Encoder rightOdometer;
    public static Encoder leftOdometer;
    public static Encoder backOdometer;

    public static DcMotor frontLeftMotor;
    public static DcMotor frontRightMotor;
    public static DcMotor backLeftMotor;
    public static DcMotor backRightMotor;
    public static DcMotor elevatorMotorRight;
    public static DcMotor elevatorMotorLeft;
    public static DcMotor winchMotorRight;
    public static DcMotor winchMotorLeft;
    public static Servo clawServo;
    public static Servo armServoRight;
    public static Servo armServoLeft;
    public static Servo wristServo;
    public static Servo rightLight;
    public static Servo leftLight;

    public static void GetHardware(HardwareMap hardwareMap)
    {
    }

    private static void GetFromHardwareMap(HardwareMap hardwareMap)
    {
        // Drive Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // EX: 0
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 0
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // EX: 1
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); // 1

        elevatorMotorRight = hardwareMap.get(DcMotor.class,"elevatorMotorRight");  // 3
        elevatorMotorLeft = hardwareMap.get(DcMotor.class,"elevatorMotorLeft"); // EX: 3

        winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight"); // 2
        winchMotorLeft =  hardwareMap.get(DcMotor.class,"winchMotorLeft"); // EX: 2

        armServoRight = hardwareMap.get(Servo.class,"armServoRight");
        armServoLeft = hardwareMap.get(Servo.class,"armServoLeft");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        wristServo = hardwareMap.get(Servo.class,"wristServo");

        leftLight = hardwareMap.get(Servo.class,"leftLight"); // 4, or 5
        rightLight = hardwareMap.get(Servo.class,"rightLight"); // 4, or 5

        // Custom Encoder Module
        leftOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        rightOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_right"));
        backOdometer = new Encoder(hardwareMap.get(DcMotor.class, "back_left"));
    }


    private static void Configure()
    {

        // Drive Motors
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        elevatorMotorRight.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        winchMotorRight.setDirection(DcMotor.Direction.REVERSE);
        winchMotorLeft.setDirection(DcMotor.Direction.FORWARD);


        armServoLeft.setDirection(Servo.Direction.REVERSE);
    }

}
