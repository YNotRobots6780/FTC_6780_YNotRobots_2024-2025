/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ColorSensorEx;
import org.firstinspires.ftc.teamcode.core.Encoder;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Odomatry Auto", group="Robot")
// @Disabled
public class FancyAuto extends LinearOpMode
{

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();



    private ColorSensorEx frontIntakeColorSensor;
    int driveTime = 0;


    // Elevator
    public int targetElevatorPosition;
    public Boolean inLine = false;
    public int frontRight = frontRightMotor.getCurrentPosition();
    public int frontLeft = frontLeftMotor.getCurrentPosition();
    public int back = backLeftMotor.getCurrentPosition();

    public int F_B = frontRight + frontLeft / 2;
    public int L_R = back;

    int sub1 = 0;
    int sub2 = 0;
    int sample1 = 0;
    int sample2 = 0;
    int RR = 0 ;
    int LR = 0 ;
    int currentPoshion = F_B + L_R;
    int sa1 = 0;
    int sa2 = 0;
    int su1 = 0;
    int su2 = 0;
    int rotashionPart1 = 0;
    boolean isRotating = false;














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

    int targetPoshion_F_B = 10000;
    int targetPoshion_L_R = 10000;
    int rotashion = 0;
    int slowingMathLR = targetPoshion_L_R - L_R;
    int slowingMathFB = targetPoshion_F_B - F_B;
    int slowingMathNegativeLR = targetPoshion_L_R + L_R;
    int slowingMathNegativeFB = targetPoshion_F_B + F_B;




    @Override
    public void runOpMode() {

        // Constants.CURRENT_TEAM = Constants.Team.Red;


        telemetry.addData(">", "Ready to Run");
        waitForStart();


    }

    public void pathFindeing()
    {

        int F_B = frontRight + frontLeft / 2;
        int L_R = back;

        telemetry.addData(">", F_B);
        telemetry.addData(">", L_R);







          if (F_B == targetPoshion_F_B)
    {
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }
    else if (F_B > targetPoshion_F_B)
    {
        backLeftMotor.setPower(  -0.5);
        frontLeftMotor.setPower( -0.5);
        backRightMotor.setPower( -0.5);
        frontRightMotor.setPower( -0.5);
    }
    else if (F_B < targetPoshion_F_B)
    {
        backLeftMotor.setPower( 0.5);
        frontLeftMotor.setPower( 0.5);
        backRightMotor.setPower( 0.5);
        frontRightMotor.setPower( 0.5);
    }



        else if (L_R > targetPoshion_L_R)
        {
            // Shimmy Right
            frontRightMotor.setPower( + 0.5);
            backRightMotor.setPower( + -0.5);
            frontLeftMotor.setPower( + -0.5);
            backLeftMotor.setPower( + -0.5);
        }
        else if (L_R < targetPoshion_L_R)
        {
            //shimmy Left
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(-0.5);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(-0.5);
        }
        else if (L_R == targetPoshion_L_R)
        {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        }

    }














    public void rotate (String deirection)
    {


        int rotashion = L_R - rotashionPart1;
        if (currentPoshion == sample1)
        {
            if (rotashion > sa1)
            {
                backLeftMotor.setPower( 0.5);
                frontLeftMotor.setPower( 0.5);
                backRightMotor.setPower(- 0.5);
                frontRightMotor.setPower(- 0.5);
            }
            else if (rotashion < sa1)
            {
                backLeftMotor.setPower( -0.5);
                frontLeftMotor.setPower( -0.5);
                backRightMotor.setPower( 0.5);
                frontRightMotor.setPower( 0.5);
            }
            else
            {

            }

        }
        else if( sample2 == currentPoshion)
        {

            //pick up sample
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower( 0);
            backRightMotor.setPower(0);
            frontRightMotor.setPower(0);
        }
        else if (sub1 == currentPoshion)
        {
            if( rotashion > su1)
            {
                backLeftMotor.setPower( 0.5);
                frontLeftMotor.setPower( 0.5);
                backRightMotor.setPower(- 0.5);
                frontRightMotor.setPower(- 0.5);
            }
            else if (rotashion < su1)
            {
                backLeftMotor.setPower( -0.5);
                frontLeftMotor.setPower( -0.5);
                backRightMotor.setPower( 0.5);
                frontRightMotor.setPower( 0.5);
            }
            else
            {

            }


        }
        else if( sub2 == currentPoshion )
        {
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontRightMotor.setPower(0);
        }




        if (isRotating == false)
        {
            rotashionPart1 = F_B;
        }
        else if (currentPoshion == sample1)
        {
            isRotating = true;
        }
        else if (currentPoshion == sub1)
        {
            isRotating = false;
        }
        else
        {
            isRotating = false;
        }


    }




    private void GetFromHardwareMap()
    {
        // Drive Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // EX: 0
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 3
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // EX: 1
        backRightMotor  = hardwareMap.get(DcMotor.class, "back_right"); // 2

        elevatorMotorRight = hardwareMap.get(DcMotor.class,"elevatorMotorRight"); // 0
        elevatorMotorLeft = hardwareMap.get(DcMotor.class,"elevatorMotorLeft"); // EX: 3

        winchMotorRight =  hardwareMap.get(DcMotor.class,"winchMotorRight"); // 1
        winchMotorLeft =  hardwareMap.get(DcMotor.class,"winchMotorLeft"); // EX: 2

        armServoRight = hardwareMap.get(Servo.class,"armServoRight");//1
        armServoLeft = hardwareMap.get(Servo.class,"armServoLeft");//2
        clawServo = hardwareMap.get(Servo.class,"clawServo");//3
        wristServo = hardwareMap.get(Servo.class,"wristServo");//0

        // Custom Encoder Module
        leftOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        rightOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_right"));
        backOdometer = new Encoder(hardwareMap.get(DcMotor.class, "back_left"));
    }

    private void Configure()
    {

        // Drive Motors
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

        elevatorMotorRight.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        winchMotorRight.setDirection(DcMotor.Direction.FORWARD);
        winchMotorLeft.setDirection(DcMotor.Direction.REVERSE);


        armServoLeft.setDirection(Servo.Direction.REVERSE);

        elevatorMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        elevatorMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotorRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        winchMotorLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        winchMotorLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        winchMotorRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }

}