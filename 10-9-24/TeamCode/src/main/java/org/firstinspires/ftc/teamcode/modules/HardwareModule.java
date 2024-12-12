package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;

public class HardwareModule {

    private static boolean hasGottenHardware;

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


    public static void GetHardware(OpMode opMode)
    {
        if (hasGottenHardware)
        {
            return;
        }

        hasGottenHardware = true;

        GetFromHardwareMap(opMode.hardwareMap);

        Configure();
    }

    private static void GetFromHardwareMap(HardwareMap hardwareMap)
    {
        // Drive Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); //
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); //
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); //
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); //

        elevatorMotorRight = hardwareMap.get(DcMotor.class,"elevatorMotorRight");
        elevatorMotorLeft = hardwareMap.get(DcMotor.class,"elevatorMotorLeft");

        winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight");
        winchMotorLeft =  hardwareMap.get(DcMotor.class,"winchMotorLeft");

        armServoRight = hardwareMap.get(Servo.class,"armServoRight");
        armServoLeft = hardwareMap.get(Servo.class,"armServoLeft");
        clawServo = hardwareMap.get(Servo.class,"clawServo");
        wristServo = hardwareMap.get(Servo.class,"wristServo");

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
