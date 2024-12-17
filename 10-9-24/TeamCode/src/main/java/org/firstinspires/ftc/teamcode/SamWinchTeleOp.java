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
import org.firstinspires.ftc.teamcode.modules.DriveModule;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;
/*

@TeleOp(name="6780 Sam's Winch TeleOp", group="Robot")
public class SamWinchTeleOp extends OpMode
{
    private DriveModule driveModule;

    // ========================================== override control ==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride = false;

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


    @Override
    public void init() {
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

        int slidePosition = elevatorMotorRight.getCurrentPosition();
        int winchPosition = winchMotorRight.getCurrentPosition();


        // =============================================== Override Toggle ===============================================
        if (gamepad2.back || gamepad1.back)
        {

            if (!isCurrentlySwitchingOverride)
            {
                isCurrentlySwitchingOverride = true;
                if (isOnOverride)
                {
                    isOnOverride = false;

                    driveModule.SetControlBehavior(DriveModule.ControlBehavior.FirstController);
                }
                else
                {
                    isOnOverride = true;

                    driveModule.SetControlBehavior(DriveModule.ControlBehavior.SecondController);
                }
            }

        }
        else
        {
            isCurrentlySwitchingOverride = false;
        }



        if (!isOnOverride)
        {
            Normall();

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
        PowerMotors(gamepad1.left_stick_x, gamepad1.left_stick_y, -gamepad1.right_stick_x);
    }
    private void Overrride()
    {
        // Drive Module powers the drive Code
        PowerMotors(gamepad2.left_stick_x, gamepad2.left_stick_y, -gamepad2.right_stick_x);
    }

    public void sevoX()
    {
        double servoPoshion = gamepad1.left_trigger;
        clawServo.setPosition(servoPoshion);
    }






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

    private void Configure()
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


        // HardwareModule.armServoLeft.setDirection(Servo.Direction.REVERSE);
    }
}
*/