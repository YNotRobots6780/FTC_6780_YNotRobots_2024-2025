package org.firstinspires.ftc.teamcode.modules;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

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

    }

}
