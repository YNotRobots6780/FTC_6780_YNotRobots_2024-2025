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
    public static DcMotor elavatorMotorRight;
    public static DcMotor elavatorMotorLeft;
    public static DcMotor winchMotorRight;
    public static DcMotor WinchMotorLeft;
    public static DcMotor intakeMotor;
    public static Servo clawGrabbingservo;
    public static Servo clawServoY1;
    public static Servo clawServoY2;
    public static Servo clawServox;
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
        elavatorMotorRight = hardwareMap.get(DcMotor.class,"elavatorMotorRight");
        elavatorMotorLeft = hardwareMap.get(DcMotor.class,"elavatorMotorLeft");
        winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight");
        WinchMotorLeft  =  hardwareMap.get(DcMotor.class,"getWinchMotorLeft");
        intakeMotor = hardwareMap.get(DcMotor.class,"justForCodeingPerposes");
        clawServoY1 = hardwareMap.get(Servo.class,"servox1");
        clawServoY2 = hardwareMap.get(Servo.class,"sevox2");
        clawGrabbingservo = hardwareMap.get(Servo.class,"sevoGrab");
        clawServox = hardwareMap.get(Servo.class,"servoY");

        // Custom Encoder Module
        leftOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        rightOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_right"));
        backOdometer = new Encoder(hardwareMap.get(DcMotor.class, "back_left"));
    }


    private static void Configure()
    {

        // Drive Motors
        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        elavatorMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        elavatorMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        winchMotorRight.setDirection(DcMotorSimple.Direction.FORWARD);
        WinchMotorLeft.setDirection(DcMotorSimple.Direction.FORWARD);




    }

}
