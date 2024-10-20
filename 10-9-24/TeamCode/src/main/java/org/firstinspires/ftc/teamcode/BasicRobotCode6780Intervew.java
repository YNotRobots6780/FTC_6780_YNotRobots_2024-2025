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
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/*
 * This OpMode executes a Mechinum Drive control TeleOp a direct drive robot
 * The code is structured as an Iterative OpMode
 *
 * In this mode, the left joystick control movement.
 * The right joystick controls rotation.
 * The A button powers the Intake.
 * The B button launches the drones
 * The Left trigger raises the winch, Left bumper Lowers the winch
 * The Right trigger extends the elevator, Right bumper retracts the elevator
 * D-Pad Down Lowers the bucket, D-Pad Left Flatens the bucket, D-Pad Up Raises the bucket
 *
 */

@TeleOp(name="6780 New robot code!", group="Robot")
public class BasicRobotCode6780Intervew extends OpMode
{
    //=====================overide controll==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwichingOverride =false;

    //====================Intake loop contoll==================================================================

    private boolean firstTimeIntake = false;
    private boolean shouldPowerIntake = false;

    //==============Servo loop controll========================================================================

    private boolean servoOpen = false;
    private boolean servoClosed = false;
    private boolean servoLoopBreak = false;
    private boolean servoFirstTime = false;



    /* Declare OpMode members. */
    private DcMotor intakeMotor;
    private DcMotor intakeLiftMotor;
    private DcMotor elavatorMotor;
    private Servo clawOpenAndClose;
    private ColorSensor frontIntakeColorSensor;

    // ===================================================================== EDIT THIS STUFF HERE!!! ======================================================================

    private static final double MOVEMENT_SPEED = 1;

    // ====================================================================================================================================================================


    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors

        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        elavatorMotor = hardwareMap.get(DcMotor.class, "elavatorMotor");
        intakeLiftMotor= hardwareMap.get(DcMotor.class, "intakeLiftMotor");
        clawOpenAndClose = hardwareMap.get(Servo.class,"clawOpenAndClose");
        frontIntakeColorSensor = hardwareMap.get(ColorSensor.class, "ENTER IN THE FRONT INTAKE MOTOR COLOR SENSOR CONFIGURATION NAME HERE!!!");



        elavatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elavatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);




        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        elavatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        intakeLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);

        // Send telemetry message to signify robot waiting;
        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    /*
     * Code to run REPEATEDLY after the driver hits INIT, but before they hit PLAY
     */
    @Override
    public void init_loop() {
    }

    /*
     * Code to run ONCE when the driver hits PLAY
     */
    @Override
    public void start() {
    }

    /*
     * Code to run REPEATEDLY after the driver hits PLAY but before they hit STOP
     */
    @Override
    public void loop() {

        if (gamepad2.back || gamepad1.back)
        {

            if (isCurrentlySwichingOverride == false)
            {
                isCurrentlySwichingOverride = true;
                if (isOnOverride == true)
                {
                    isOnOverride = false;
                    elavatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elavatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    isOnOverride = true;
                    elavatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intakeLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

        }
        else
        {
            isCurrentlySwichingOverride = false;
        }



        if ( isOnOverride == true)
        {

            if(gamepad2.a)
            {
                intakeMotor.setPower(1);
            }

            if (gamepad2.b)
            {
                intakeLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


                intakeMotor.setPower(1);
            }

            if(gamepad2.y)
            {
                intakeLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                intakeMotor.setPower(1);
                intakeLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }


            if(gamepad2.right_bumper)
            {
                elavatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                elavatorMotor.setPower(1);
            }

            if (gamepad2.right_trigger > 0.5)
            {
                elavatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                elavatorMotor.setPower(1);
                elavatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if(gamepad2.left_bumper)
            {
                elavatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                elavatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }
        else
        {

            // a= intke b=intake up y= intake down



            if (gamepad1.a)
            {

                if (firstTimeIntake == false)
                {
                    firstTimeIntake = true;
                    if (shouldPowerIntake)
                    {
                        shouldPowerIntake = false;
                    }
                    else
                    {
                        shouldPowerIntake = true;
                    }
                }

            }
            else
            {
                firstTimeIntake = false;
            }

            if(shouldPowerIntake == true)
            {
                intakeMotor.setPower(1);
            }

            if(gamepad1.b)
            {
                intakeLiftMotor.setPower(1);
                intakeLiftMotor.setTargetPosition(0);
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.y)
            {
                intakeLiftMotor.setPower(1);
                intakeLiftMotor.setTargetPosition(0);
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.dpad_down)
            {
                elavatorMotor.setPower(1);
                elavatorMotor.setTargetPosition(Constants.lowBucket);
                elavatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if(gamepad1.dpad_right)
            {
                elavatorMotor.setPower(1);
                elavatorMotor.setTargetPosition(Constants.highBucket);
                elavatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if(gamepad1.dpad_left)
            {
                elavatorMotor.setPower(1);
                elavatorMotor.setTargetPosition(Constants.lowSample);
                elavatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }




            if(gamepad1.dpad_up)
            {
                elavatorMotor.setPower(1);
                elavatorMotor.setTargetPosition(Constants.highSample);
                elavatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if(gamepad1.left_bumper)
            {
                elavatorMotor.setPower(1);
                elavatorMotor.setTargetPosition(0);
                elavatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            if(servoOpen==true)
            {
                clawOpenAndClose.setPosition(Constants.clawOpen);
            }


            if (servoClosed==true)
            {
                clawOpenAndClose.setPosition(Constants.clawclosed);
            }

        }



    }


    public void ToggleLoops() {

//==============================================ServoToggle=====================================================================
        if (gamepad1.x) {

            if (servoFirstTime == false) {
                servoFirstTime = true;
                if (servoFirstTime == true) {
                    servoFirstTime = false;

                    servoOpen = true;

                } else {
                    servoClosed = true;
                    servoFirstTime = true;
                }
            }

        } else {
            servoFirstTime = false;
        }

//====================================================IntakeToggle================================================

        if (gamepad1.a) {

            if (firstTimeIntake == false) {
                firstTimeIntake = true;
                if (shouldPowerIntake == true) {
                    shouldPowerIntake = false;

                } else {
                    shouldPowerIntake = true;
                }
            }

        } else {
            firstTimeIntake = false;
        }

        if (shouldPowerIntake == true) {
            intakeMotor.setPower(1);
        }

    }
        /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
