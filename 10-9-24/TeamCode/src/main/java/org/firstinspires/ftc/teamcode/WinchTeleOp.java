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

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;
import org.firstinspires.ftc.teamcode.modules.SlideModule;


@TeleOp(name="6780 Winch Teleop", group="Robot")
public class WinchTeleOp extends OpMode
{
    private enum ScorePosition
    {
        HighBasket,
        LowBasket,
        HighChamber,
        LowChamber,
        NOT_SCORING,
    }
    // ========================================== override control ==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride =false;

    // ========================================== override control ==========================================

    private ScorePosition scorePosition;
    private boolean isWinchShootingHigh;
    private boolean isWinchScoringSamples;




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



    @Override
    public void init() {

        scorePosition = ScorePosition.NOT_SCORING;
        GetFromHardwareMap();

        Configure();

        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start()  {

    }

    @Override
    public void loop() {


        // =============================================== Override Toggle ===============================================
        if (gamepad2.back || gamepad1.back)
        {

            if (!isCurrentlySwitchingOverride)
            {
                isCurrentlySwitchingOverride = true;
                isOnOverride = !isOnOverride;
            }
        }
        else
        {
            isCurrentlySwitchingOverride = false;
        }



        if (isOnOverride)
        {
            Normall();
        }
        else
        {
            Overrride();
        }

        telemetry.update();



    }

    @Override
    public void stop() {

    }



    private void Normall()
    {
        telemetry.addData(">", elevatorMotorRight.getCurrentPosition());
        telemetry.addData(">", winchMotorRight.getCurrentPosition());
        telemetry.addData(">", elevatorMotorRight.getTargetPosition());
        telemetry.addData(">", winchMotorRight.getTargetPosition());
        telemetry.update();
        telemetry.addData("clawServo", (clawServo.getPortNumber()));
        telemetry.addData("arm right", (armServoLeft.getPortNumber()));
        telemetry.addData("arm left", (armServoRight.getPortNumber()));
        telemetry.addData("wrist", (wristServo.getPortNumber()));

        PowerDriveMotors(-gamepad1.left_stick_x, gamepad1.left_stick_y * 1.1, -gamepad1.right_stick_x);

        if (gamepad1.dpad_down)
        {
            scorePosition = ScorePosition.NOT_SCORING;
            SetWinchPosition(100);
            SetElevatorPosition(50);
        }

        else if (gamepad1.dpad_left)
        {
            if (!isWinchScoringSamples)
            {
                isWinchScoringSamples = true;
                isWinchShootingHigh = false;
            }

            if (isWinchShootingHigh)
            {
                isWinchShootingHigh = false;
                scorePosition = ScorePosition.LowBasket;
            }
            else
            {
                isWinchShootingHigh = true;
                scorePosition = ScorePosition.HighBasket;
            }
        }
        else if (gamepad1.dpad_right)
        {
            if (isWinchScoringSamples)
            {
                isWinchScoringSamples = false;
                isWinchShootingHigh = false;
            }

            if (isWinchShootingHigh)
            {
                isWinchShootingHigh = false;
                scorePosition = ScorePosition.LowChamber;
            }
            else
            {
                isWinchShootingHigh = true;
                scorePosition = ScorePosition.HighChamber;
            }
        }

       if(gamepad1.a)
       {
           clawServo.setPosition(1);
       }
        if(gamepad1.b)
        {
            armServoRight.setPosition(0);
        }
        if(gamepad1.y)
        {
            armServoLeft.setPosition(0);
        }
        if(gamepad1.x)
        {
            wristServo.setPosition(0);
        }


           switch (scorePosition)
        {
            case NOT_SCORING:
                if (gamepad1.right_bumper)
                {
                    SetElevatorPosition(elevatorMotorLeft.getCurrentPosition() - 5);
                }
                else if (gamepad1.right_trigger > 0.25)
                {
                    SetElevatorPosition(elevatorMotorLeft.getCurrentPosition() + 5);
                }

                if (gamepad1.right_bumper)
                {
                    SetElevatorPosition(elevatorMotorLeft.getCurrentPosition() - 5);
                }
                else if (gamepad1.right_trigger > 0.25)
                {
                    SetElevatorPosition(elevatorMotorLeft.getCurrentPosition() + 5);
                }
                break;
            case LowBasket:
                SetWinchPosition(Constants.WINCH_LOW_BASKET);
                SetWinchPower(0.5);
                SetElevatorPosition(Constants.ELEVATOR_LOW_BASKET);
                SetElevatorPower(1);
              //   armServoLeft.setPosition(Constants.clawBaskit);
              //   armServoRight.setPosition(Constants.clawBaskit);
                break;
            case HighBasket:
                SetWinchPosition(Constants.WINCH_HIGH_BASKET);
                SetWinchPower(0.5);
                SetElevatorPosition(Constants.ELEVATOR_HIGH_BASKET);
                SetElevatorPower(1);
              //  armServoLeft.setPosition(Constants.clawBaskit);
              //  armServoRight.setPosition(Constants.clawBaskit);
                break;
            case LowChamber:
                SetWinchPosition(Constants.WINCH_LOW_CHAMBER);
                SetWinchPower(0.5);
                SetElevatorPosition(Constants.ELEVATOR_LOW_CHAMBER);
                SetElevatorPower(1);
               // armServoLeft.setPosition(Constants.clawChaber);
               // armServoRight.setPosition(Constants.clawChaber);
                break;
            case HighChamber:
                SetWinchPosition(Constants.WINCH_HIGH_CHAMBER);
                SetWinchPower(0.5);
                SetElevatorPosition(Constants.ELEVATOR_HIGH_CHAMBER);
                SetElevatorPower(1);
               // armServoLeft.setPosition(Constants.clawChaber);
              //  armServoRight.setPosition(Constants.clawChaber);

                break;
        }
    }
    private void Overrride()
    {
        PowerDriveMotors(-gamepad2.left_stick_x, gamepad2.left_stick_y * 1.1, -gamepad2.right_stick_x);
    }



    private void PowerDriveMotors(double x, double z, double rotation)
    {
        double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);

        frontLeftMotor.setPower(((z + x + rotation) / denominator) * Constants.DRIVE_SPEED);
        frontRightMotor.setPower(((z - x - rotation) / denominator) * Constants.DRIVE_SPEED);
        backLeftMotor.setPower(((z - x + rotation) / denominator) * Constants.DRIVE_SPEED);
        backRightMotor.setPower(((z + x - rotation) / denominator) * Constants.DRIVE_SPEED);
    }

    private void SetWinchPower(double power)
    {
        winchMotorLeft.setPower(power);
        winchMotorRight.setPower(power);
    }

    private void SetWinchPosition(int position)
    {
        winchMotorLeft.setTargetPosition(position);
        winchMotorRight.setTargetPosition(position);

        winchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void SetElevatorPower(double power)
    {
        elevatorMotorLeft.setPower(power);
        elevatorMotorRight.setPower(power);
    }
    private void SetElevatorPosition(int position)
    {
        elevatorMotorLeft.setTargetPosition(position);
        elevatorMotorRight.setTargetPosition(position);

        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void GetFromHardwareMap()
    {
        // Drive Motors
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left"); // EX: 0
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right"); // 3
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left"); // EX: 1
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right"); // 2

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


        // armServoLeft.setDirection(Servo.Direction.REVERSE);

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
