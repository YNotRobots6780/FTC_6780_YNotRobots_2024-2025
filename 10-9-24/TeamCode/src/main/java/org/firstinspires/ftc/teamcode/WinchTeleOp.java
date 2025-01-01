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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;
import org.firstinspires.ftc.teamcode.core.Timer;


/// adb connect 192.168.43.1:5555



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


    private Timer timer;

    // ========================================== override control ==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride = false;

    // ========================================== Score Positions ==========================================

    private ScorePosition scorePosition;
    private boolean isWinchShootingHigh;
    private boolean isWinchScoringSamples;
    private boolean isDPadPressed;

    // ========================================== Claw ==========================================
    private boolean isClawPressed;
    private boolean isClawOpen;
    private boolean slideControl = false;
    private boolean isSafeToExtend = true;

    // ========================================== Drive ==========================================
    private boolean isPressingSlowDrive;
    private boolean isDrivingSlowly;




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

        armServoLeft.setPosition(Constants.armUp);
        armServoRight.setPosition(Constants.armUp);

    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start()  {
        timer = new Timer();
    }

    @Override
    public void loop() {
        timer.Update();

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



        if (!isOnOverride)
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

        if (gamepad1.right_stick_button || gamepad1.left_stick_button)
        {
            if (!isPressingSlowDrive)
            {
                isPressingSlowDrive = true;
                isDrivingSlowly = !isDrivingSlowly;
            }
        }
        else
        {
            isPressingSlowDrive = false;
        }

        if (isDrivingSlowly)
        {
            PowerDriveMotors(-gamepad1.left_stick_x, gamepad1.left_stick_y * 1.1, -gamepad1.right_stick_x, Constants.SLOW_DRIVE_SPEED);
        }
        else
        {
            PowerDriveMotors(-gamepad1.left_stick_x, gamepad1.left_stick_y * 1.1, -gamepad1.right_stick_x, Constants.DRIVE_SPEED);
        }

        if (gamepad1.dpad_down)
        {
            slideControl = true;
            scorePosition = ScorePosition.NOT_SCORING;
            isSafeToExtend = false;
        }
        else if (gamepad1.dpad_left)
        {
            isSafeToExtend = false;
            if (!isWinchScoringSamples)
            {
                isWinchScoringSamples = true;
                isWinchShootingHigh = false;
            }

            if (!isDPadPressed)
            {
                isDPadPressed = true;
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
        }
        else if (gamepad1.dpad_right)
        {
            isSafeToExtend = false;
            if (isWinchScoringSamples)
            {
                isWinchScoringSamples = false;
                isWinchShootingHigh = false;
            }

            if (!isDPadPressed)
            {
                isDPadPressed = true;
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
        }
        else
        {
            isDPadPressed = false;
        }




        switch (scorePosition)
        {
            case NOT_SCORING:
                telemetry.addData("NOT SCORING", "");

               if (slideControl)
               {
                   if (elevatorMotorLeft.getCurrentPosition() < 150 && elevatorMotorRight.getCurrentPosition() < 150)
                   {
                       SetWinchPosition(100);
                       slideControl = false;
                   }
                   SetElevatorPosition(50);
               }

                if (gamepad1.left_bumper)
                {
                    SetElevatorPosition(elevatorMotorLeft.getTargetPosition() - (int)(1000 * timer.deltaTime));
                    SetWinchPosition(100 + (int)(elevatorMotorLeft.getCurrentPosition() * Constants.GRAB_WINCH_TO_ELEVATOR_RATIO));
                    SetElevatorPower(1);
                }
                else if (gamepad1.left_trigger > 0.25)
                {
                    SetElevatorPosition(elevatorMotorLeft.getTargetPosition() + (int)(1000 * timer.deltaTime));
                    SetWinchPosition(100 + (int)(elevatorMotorLeft.getCurrentPosition() * Constants.GRAB_WINCH_TO_ELEVATOR_RATIO));
                    SetElevatorPower(1);
                }
                break;
            case LowBasket:

                MoveWinchAndElevator(Constants.WINCH_LOW_BASKET, Constants.ELEVATOR_LOW_BASKET);
                // SetWinchPosition(Constants.WINCH_LOW_BASKET);
                // SetElevatorPosition(Constants.ELEVATOR_LOW_BASKET);
                SetElevatorPower(1);
                //   armServoLeft.setPosition(Constants.clawBaskit);
                //   armServoRight.setPosition(Constants.clawBaskit);
                break;
            case HighBasket:
                MoveWinchAndElevator(Constants.WINCH_HIGH_BASKET, Constants.ELEVATOR_HIGH_BASKET);
                // SetWinchPosition(Constants.WINCH_HIGH_BASKET);
                // SetElevatorPosition(Constants.ELEVATOR_HIGH_BASKET);
                SetElevatorPower(1);
                //  armServoLeft.setPosition(Constants.clawBaskit);
                //  armServoRight.setPosition(Constants.clawBaskit);
                break;
            case LowChamber:
                MoveWinchAndElevator(Constants.WINCH_LOW_CHAMBER, Constants.ELEVATOR_LOW_CHAMBER);
                // SetWinchPosition(Constants.WINCH_LOW_CHAMBER);
                // SetElevatorPosition(Constants.ELEVATOR_LOW_CHAMBER);
                SetElevatorPower(1);
                // armServoLeft.setPosition(Constants.clawChaber);
                // armServoRight.setPosition(Constants.clawChaber);
                break;
            case HighChamber:
                MoveWinchAndElevator(Constants.WINCH_HIGH_CHAMBER, Constants.ELEVATOR_HIGH_CHAMBER);

                // SetWinchPosition(Constants.WINCH_HIGH_CHAMBER);
                // SetElevatorPosition(Constants.ELEVATOR_HIGH_CHAMBER);
                SetElevatorPower(1);
                // armServoLeft.setPosition(Constants.clawChaber);
                //  armServoRight.setPosition(Constants.clawChaber);
                break;
        }

        if(gamepad1.right_trigger > 0.1)
        {
            wristServo.setPosition(gamepad1.right_trigger * Constants.WRIST_MODIFIER);
        }
        else
        {
            wristServo.setPosition(Constants.WRIST_DEFAULT_POSITION);
        }

        if (gamepad1.a)
        {
            if (!isClawPressed)
            {
                isClawPressed = true;
                isClawOpen = !isClawOpen;
            }
        }
        else
        {
            isClawPressed = false;
        }

        telemetry.addData("<", "" + clawServo.getPosition());
        if (isClawOpen)
        {
            // SetWinchPosition(winchMotorLeft.getCurrentPosition() - 50);
            clawServo.setPosition(Constants.CLAW_OPEN_POSITION);
        }
        else
        {
            clawServo.setPosition(Constants.CLAW_CLOSE_POSITION);
        }

        if (gamepad1.x)
        {
            armServoLeft.setPosition(Constants.armUp);
            armServoRight.setPosition(Constants.armUp);
        }
        if (gamepad1.y)
        {
            armServoLeft.setPosition(Constants.armMid);
            armServoRight.setPosition(Constants.armMid);
        }
        if (gamepad1.b)
        {
            if ((scorePosition == ScorePosition.NOT_SCORING && elevatorMotorLeft.getCurrentPosition() > 200) || scorePosition == ScorePosition.HighChamber)
            {
                armServoLeft.setPosition(Constants.armStraightDown);
                armServoRight.setPosition(Constants.armStraightDown);
            }
            else
            {
                armServoLeft.setPosition(Constants.armPickUp);
                armServoRight.setPosition(Constants.armPickUp);
            }
        }
    }

    private void Overrride()
    {
        if (gamepad2.right_stick_button || gamepad2.left_stick_button)
        {
            if (!isPressingSlowDrive)
            {
                isPressingSlowDrive = true;
                isDrivingSlowly = !isDrivingSlowly;
            }
        }
        else
        {
            isPressingSlowDrive = false;
        }

        if (isDrivingSlowly)
        {
            PowerDriveMotors(-gamepad2.left_stick_x, gamepad2.left_stick_y * 1.1, -gamepad2.right_stick_x, Constants.SLOW_DRIVE_SPEED);
        }
        else
        {
            PowerDriveMotors(-gamepad2.left_stick_x, gamepad2.left_stick_y * 1.1, -gamepad2.right_stick_x, Constants.DRIVE_SPEED);
        }


        if (gamepad2.left_trigger > 0.25)
        {
            SetElevatorPower(1);
        }
        else if (gamepad1.left_bumper)
        {

        }
    }



    private void PowerDriveMotors(double x, double z, double rotation, double speed)
    {
        double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);

        frontLeftMotor.setPower(((z + x + rotation) / denominator) * speed);
        frontRightMotor.setPower(((z - x - rotation) / denominator) * speed);
        backLeftMotor.setPower(((z - x + rotation) / denominator) * speed);
        backRightMotor.setPower(((z + x - rotation) / denominator) * speed);
    }


    private void MoveWinchAndElevator(int winchPosition, int elevatorPosition)
    {
        if (!isSafeToExtend)
        {
            if (Math.abs(winchMotorLeft.getCurrentPosition() - winchPosition) > 100)
            {
                if (elevatorMotorLeft.getCurrentPosition() > 300)
                {
                    // Needs to move elevator in first
                    SetElevatorPosition(200);
                }
                else
                {
                    isSafeToExtend = true;
                    SetWinchPosition(winchPosition);
                    SetElevatorPosition(elevatorPosition);
                }
            }
            else
            {
                isSafeToExtend = true;
                SetWinchPosition(winchPosition);
                SetElevatorPosition(elevatorPosition);
            }
        }
        else
        {
            SetWinchPosition(winchPosition);
            SetElevatorPosition(elevatorPosition);
        }
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

        elevatorMotorRight.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotorLeft.setDirection(DcMotor.Direction.FORWARD);
        winchMotorRight.setDirection(DcMotor.Direction.FORWARD);
        winchMotorLeft.setDirection(DcMotor.Direction.REVERSE);


        armServoRight.setDirection(Servo.Direction.REVERSE);

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

        winchMotorLeft.setPower(Constants.WINCH_SPEED);
        winchMotorRight.setPower(Constants.WINCH_SPEED);
        winchMotorRight.setTargetPosition(0);
        winchMotorLeft.setTargetPosition(0);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
}
