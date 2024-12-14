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
<<<<<<< Updated upstream
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;
import org.firstinspires.ftc.teamcode.core.Timer;
=======
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.Encoder;
import org.firstinspires.ftc.teamcode.modules.DriveModule;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;
>>>>>>> Stashed changes


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

<<<<<<< Updated upstream
    // ========================================== Score Positions ==========================================

    private ScorePosition scorePosition;
    private boolean isWinchShootingHigh;
    private boolean isWinchScoringSamples;
    private boolean isDPadPressed;

    // ========================================== Claw ==========================================
    private boolean isClawPressed;
    private boolean isClawOpen;






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

=======
    // ========================================== Claw Toggle ==========================================
    private boolean isClawBeingPressed;
    private boolean isClawOpen = false;


    public Encoder rightOdometer;
    public Encoder leftOdometer;
    public Encoder backOdometer;

    public DcMotor frontLeftMotor;
    public DcMotor frontRightMotor;
    public DcMotor backLeftMotor;
    public DcMotor backRightMotor;
    public DcMotor elevatorMotorRight;
    public DcMotor elevatorMotorLeft;
    public DcMotor winchMotorRight;
    public DcMotor winchMotorLeft;

    public Servo clawServo;
    public Servo armServoRight;
    public Servo armServoLeft;
    public Servo wristServo;
    public Servo rightLight;
    public Servo leftLight;
>>>>>>> Stashed changes


    @Override
    public void init() {
<<<<<<< Updated upstream

        scorePosition = ScorePosition.NOT_SCORING;
        GetFromHardwareMap();

=======
        GetFromHardwareMap();
>>>>>>> Stashed changes
        Configure();

        telemetry.addData(">", "Robot Ready.  Press Play.");    //

    }

    @Override
    public void init_loop() {

    }

    @Override
    public void start()  {
<<<<<<< Updated upstream
        timer = new Timer();
=======

>>>>>>> Stashed changes
    }

    @Override
    public void loop() {
<<<<<<< Updated upstream
        timer.Update();
=======

        int slidePosition = elevatorMotorRight.getCurrentPosition();
        int winchPosition = winchMotorRight.getCurrentPosition();

>>>>>>> Stashed changes

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
<<<<<<< Updated upstream
=======

            leftLight.setPosition(0.5);
            rightLight.setPosition(0.5);

            if(slidePosition < Constants.slideMin)
            {
                if(gamepad1.right_trigger>0.5)
                {
                    winchMotorRight.setTargetPosition(Constants.winchUp);
                    winchMotorLeft.setTargetPosition(Constants.winchUp);
                    winchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }
            }
            if (slidePosition == Constants.elavatorDown)
            {
                if(gamepad1.right_bumper)
                {
                    winchMotorRight.setTargetPosition(Constants.winchDown);
                    winchMotorLeft.setTargetPosition(Constants.winchDown);
                    winchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

            }
            if( winchPosition == Constants.winchUp || winchPosition == Constants.winchDown)
            {
                if (gamepad1.dpad_down) {
                    elevatorMotorRight.setTargetPosition(Constants.HIGH_BUCKET);
                    elevatorMotorLeft.setTargetPosition(Constants.HIGH_BUCKET);
                    elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad1.dpad_right) {
                    elevatorMotorRight.setTargetPosition(Constants.LOW_BUCKET);
                    elevatorMotorLeft.setTargetPosition(Constants.LOW_BUCKET);
                    elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


                if (gamepad1.dpad_up) {
                    elevatorMotorRight.setTargetPosition(Constants.HIGH_SAMPLE);
                    elevatorMotorLeft.setTargetPosition(Constants.HIGH_SAMPLE);
                    elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


                if (gamepad1.dpad_left) {
                    elevatorMotorRight.setTargetPosition(Constants.LOW_SAMPLE);
                    elevatorMotorLeft.setTargetPosition(Constants.LOW_SAMPLE);
                    elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad1.left_bumper) {
                    elevatorMotorRight.setTargetPosition(Constants.elavatorDown);
                    elevatorMotorLeft.setTargetPosition(Constants.elavatorDown);
                    elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

            }


            if (gamepad1.a)
            {
                if (!isClawBeingPressed)
                {
                    isClawBeingPressed = true;
                    isClawOpen = !isClawOpen;
                }
            }
            else
            {
                isClawBeingPressed = false;
            }

            if(isClawOpen)
            {
                clawServo.setPosition(Constants.clawOpen);
            }
            else
            {
                clawServo.setPosition(Constants.clawClosed);
            }



            if(slidePosition == Constants.HIGH_BUCKET || slidePosition == Constants.LOW_BUCKET)
            {
                armServoRight.setPosition(Constants.clawBucket);
                armServoLeft.setPosition(Constants.clawBucket);
            }

            else if(slidePosition == Constants.HIGH_SAMPLE || slidePosition == Constants.LOW_SAMPLE)
            {
                armServoRight.setPosition(Constants.clawSample);
                armServoLeft.setPosition(Constants.clawSample);
            }

            else if (slidePosition == Constants.elavatorDown)
            {
                armServoRight.setPosition(Constants.clawPickUp);
                armServoLeft.setPosition(Constants.clawPickUp);
                sevoX();
            }

>>>>>>> Stashed changes
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
<<<<<<< Updated upstream

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
                if (gamepad1.left_bumper)
                {
                    telemetry.addData("LEFT BUMPER", "" + elevatorMotorLeft.getTargetPosition());
                    telemetry.addData("LEFT BUMPER + 2", "" + (int)(1000 * timer.deltaTime));
                    SetElevatorPosition(elevatorMotorLeft.getTargetPosition() - (int)(1000 * timer.deltaTime));
                    SetWinchPosition(120 + (int)(elevatorMotorLeft.getCurrentPosition() * Constants.GRAB_WINCH_TO_ELEVATOR_RATIO));
                    SetWinchPower(0.5);
                    SetElevatorPower(1);
                }
                else if (gamepad1.left_trigger > 0.25)
                {
                    telemetry.addData("LEFT TRIGGER", "" + elevatorMotorLeft.getTargetPosition());
                    telemetry.addData("LEFT TRIGGER 2", "" + (int)(1000 * timer.deltaTime));
                    SetElevatorPosition(elevatorMotorLeft.getTargetPosition() + (int)(1000 * timer.deltaTime));
                    SetWinchPosition(120 + (int)(elevatorMotorLeft.getCurrentPosition() * Constants.GRAB_WINCH_TO_ELEVATOR_RATIO));
                    SetWinchPower(0.5);
                    SetElevatorPower(1);
                }
                break;
            case LowBasket:
                SetWinchPosition(Constants.WINCH_LOW_BASKET);
                SetWinchPower(0.5);
              // if(winchMotorLeft.getCurrentPosition() < Constants.WINCH_LOW_BASKET /2 && winchMotorRight.getCurrentPosition() < Constants.WINCH_LOW_BASKET)
              // {
                SetElevatorPosition(Constants.ELEVATOR_LOW_BASKET);
                SetElevatorPower(1);
                //   armServoLeft.setPosition(Constants.clawBaskit);
                //   armServoRight.setPosition(Constants.clawBaskit);
              // }
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

            clawServo.setPosition(1);
        }
        else
        {

            clawServo.setPosition(0);
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
            armServoLeft.setPosition(Constants.armPickUp);
            armServoRight.setPosition(Constants.armPickUp);
        }

        if(gamepad1.left_bumper)
        {

        }


=======
        PowerMotors(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
>>>>>>> Stashed changes
    }

    private void Overrride()
    {
<<<<<<< Updated upstream
        PowerDriveMotors(-gamepad2.left_stick_x, gamepad2.left_stick_y * 1.1, -gamepad2.right_stick_x);
=======
        // Drive Module powers the drive Code
        PowerMotors(gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x);
>>>>>>> Stashed changes
    }



    private void PowerDriveMotors(double x, double z, double rotation)
    {
<<<<<<< Updated upstream
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
=======
        double servoPoshion = gamepad1.left_trigger;
        clawServo.setPosition(servoPoshion);
>>>>>>> Stashed changes
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

<<<<<<< Updated upstream
        elevatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
=======



    private void PowerMotors(double x, double z, double rotation)
    {
        double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);

        frontLeftMotor.setPower(((z + x + rotation) / denominator));
        frontRightMotor.setPower(((z - x - rotation) / denominator));
        backLeftMotor.setPower(((z - x + rotation) / denominator));
        backRightMotor.setPower(((z + x - rotation) / denominator));
    }

    private void PowerWinch(double power)
    {
        winchMotorLeft.setPower(power);
        winchMotorRight.setPower(power);
        winchMotorLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void SetWinchPosition(int position)
    {
        winchMotorLeft.setTargetPosition(position);
        winchMotorRight.setTargetPosition(position);
        winchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
>>>>>>> Stashed changes
    }


    private void GetFromHardwareMap()
    {
        // Drive Motors
<<<<<<< Updated upstream
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
=======
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
>>>>>>> Stashed changes
    }

    private void Configure()
    {
<<<<<<< Updated upstream

=======
>>>>>>> Stashed changes
        // Drive Motors
        frontLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        frontRightMotor.setDirection(DcMotor.Direction.REVERSE);
        backLeftMotor.setDirection(DcMotor.Direction.FORWARD);
        backRightMotor.setDirection(DcMotor.Direction.REVERSE);

<<<<<<< Updated upstream
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


=======
        elevatorMotorRight.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotorLeft.setDirection(DcMotor.Direction.REVERSE);
        winchMotorRight.setDirection(DcMotor.Direction.REVERSE);
        winchMotorLeft.setDirection(DcMotor.Direction.FORWARD);


        // HardwareModule.armServoLeft.setDirection(Servo.Direction.REVERSE);
>>>>>>> Stashed changes
    }
}
