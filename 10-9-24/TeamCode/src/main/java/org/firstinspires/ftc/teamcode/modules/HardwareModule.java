package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Constants;
import org.firstinspires.ftc.teamcode.IntakeHandler;
import org.firstinspires.ftc.teamcode.core.ColorSensorEx;
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
    public static DcMotor intakeMotor;
    public static DcMotor intakeLiftMotor;
    public static DcMotor elevatorMotor;

    public static Servo intakeServo1;
    public static Servo intakeServo2;
    public static Servo clawServo;
    public static ColorSensorEx frontIntakeColorSensor;

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
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // ex: 1
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 2
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // ex: 0
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); // 3

        // Motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor"); // ex: 2
        elevatorMotor = hardwareMap.get(DcMotor.class, "elavatorMotor"); // 1
        intakeLiftMotor = hardwareMap.get(DcMotor.class, "intakeLiftMotor"); // ex 3

        // Servos
        clawServo = hardwareMap.get(Servo.class,"claw"); // 3
        intakeServo1 = hardwareMap.get(Servo.class,"intakeServo1");
        intakeServo2 = hardwareMap.get(Servo.class,"intakeServo2");

        // Custom Color Sensor Module
        frontIntakeColorSensor = new ColorSensorEx(hardwareMap.get(ColorSensor.class, "frontColorSensor")); // EX: 12C 3

        // Custom Encoder Module
        leftOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        rightOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_right"));
        backOdometer = new Encoder(hardwareMap.get(DcMotor.class, "back_left"));
    }


    private static void Configure()
    {
        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        // Drive Motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);

        // Motors
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Servos
        intakeServo1.setDirection(Servo.Direction.REVERSE);


        elevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

}
