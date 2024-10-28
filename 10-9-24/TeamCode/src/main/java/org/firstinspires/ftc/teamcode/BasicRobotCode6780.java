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
public class BasicRobotCode6780 extends OpMode
{

    //=====================Color Sensor==========================================

    private ColorSensorDetails frontIntakeColorSensorDetails;

    //=====================override control==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride =false;

    //====================Intake loop control==================================================================

    private boolean firstTimeIntake = false;
    private boolean shouldPowerIntake = false;

    //==============Servo loop control========================================================================

    private boolean servoOpen = false;
    private boolean servoClosed = false;
    private boolean servoLoopBreak = false;
    private boolean servoFirstTime = false;


    /* Declare OpMode members. */
    private DcMotor frontLeftMotor;
    private DcMotor frontRightMotor;
    private DcMotor backLeftMotor;
    private DcMotor backRightMotor;
    private DcMotor intakeMotor;
    private DcMotor intakeLiftMotor;
    private DcMotor elevatorMotor;
    private Servo clawOpenAndClose; // +++ I would go with "clawServo"
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
        frontLeftMotor = hardwareMap.get(DcMotor.class, "front_left");
        frontRightMotor = hardwareMap.get(DcMotor.class, "front_right");
        backLeftMotor = hardwareMap.get(DcMotor.class, "back_left");
        backRightMotor = hardwareMap.get(DcMotor.class, "back_right");
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor");
        elevatorMotor = hardwareMap.get(DcMotor.class, "elavatorMotor");
        intakeLiftMotor= hardwareMap.get(DcMotor.class, "intakeLiftMotor");
        clawOpenAndClose = hardwareMap.get(Servo.class,"clawOpenAndClose");
        frontIntakeColorSensor = hardwareMap.get(ColorSensor.class, "ENTER IN THE FRONT INTAKE MOTOR COLOR SENSOR CONFIGURATION NAME HERE!!!");
        frontIntakeColorSensorDetails = new ColorSensorDetails(frontIntakeColorSensor);


        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);



        frontLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        frontRightMotor.setDirection(DcMotor.Direction.FORWARD);
        backLeftMotor.setDirection(DcMotor.Direction.REVERSE);
        backRightMotor.setDirection(DcMotor.Direction.FORWARD);
        intakeMotor.setDirection(DcMotor.Direction.FORWARD);
        elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
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

        //===============================================overideToggle===================================================
        if (gamepad2.back || gamepad1.back)
        {

            if (isCurrentlySwitchingOverride == false)
            {
                isCurrentlySwitchingOverride = true;
                if (isOnOverride == true)
                {
                    isOnOverride = false;
                    elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }
                else
                {
                    isOnOverride = true;
                    elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    intakeLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                }
            }

        }
        else
        {
            isCurrentlySwitchingOverride = false;
        }



        if ( isOnOverride == true)
        {
            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            double z = -gamepad2.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double ry = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(ry), 1);
            double frontLeftPower = (z + x + ry) / denominator;
            double frontRightPower = (z - x - ry) / denominator;
            double backLeftPower = (z - x + ry) / denominator;
            double backRightPower = (z + x - ry) / denominator;

            frontLeftMotor.setPower(frontLeftPower * MOVEMENT_SPEED);
            frontRightMotor.setPower(frontRightPower * MOVEMENT_SPEED);
            backLeftMotor.setPower(backLeftPower * MOVEMENT_SPEED);
            backRightMotor.setPower(backRightPower * MOVEMENT_SPEED);

            // +++ I would not even recommend running a Servo toggle. The toggle could mess up. So I would recommend having 1 button per servo position.
            ToggleLoops();


            // +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ START INTAKE NOTES +++++++++++++++++++++++++++++++++++++++++++++++++++++++++++

            // +++ I am assuming for intaking in the Samples
            if(gamepad2.a)
            {
                intakeMotor.setPower(1);
            }

            // +++ You also need to be able to spit out the samples


            // +++ I think this is to let the intake down, or bring it up, but you use Intake Motor, Not Intake Lift Motor
            if (gamepad2.b)
            {
                // +++ It is currently forward. I would Only Recommend using setDirection in the Init Function.
                // +++ Remember, motors go from -1.0, to 1.0, and are a double so you can have decimals numbers. Servos are 0.0 to 1.0 and are doubles.
                intakeLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);


                // +++ Why is it "intakeLiftMotor" above, but "intakeMotor" right here?
                intakeMotor.setPower(1); // +++ Set this to -1 to go backwards
            }

            // +++ Same Problems as the one above
            // +++ I think this is to let the intake down, or bring it up, but you use Intake Motor, Not Intake Lift Motor
            if(gamepad2.y)
            {
                // +++ I would Only Recommend using setDirection in the Init Function. Also you don't need to Reverse then set it back to forwards.
                // +++ Remember, motors go from -1.0, to 1.0, and are a double so you can have decimals numbers. Servos are 0.0 to 1.0 and are doubles.
                intakeLiftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

                // +++ Just like the one above, This is "intakeMotor" but you reverse the "intakeLiftMotor"
                intakeMotor.setPower(1); // +++ Set this to -1 to go backwards
                intakeLiftMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }


            // ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++ END INTAKE NOTES ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++


            if(gamepad2.right_bumper)
            {
                // +++ I would Only Recommend using setDirection in the Init Function. Also you don't need to Reverse then set it back to forwards.
                elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
                elevatorMotor.setPower(1);
            }

            if (gamepad2.right_trigger > 0.5)
            {
                // +++ I would Only Recommend using setDirection in the Init Function. Also you don't need to Reverse then set it back to forwards.
                // +++ Remember, motors go from -1.0, to 1.0, and are a double so you can have decimals numbers. Servos are 0.0 to 1.0 and are doubles.
                elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
                // +++ Set this to -1 to go backwards
                elevatorMotor.setPower(1); // +++ You can use "elavatorMotor.setPower(gamepad2.right_trigger)" to make it so the more you press the button, the faster it goes
                elevatorMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            }

            if(gamepad2.left_bumper)
            {
                elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

                elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            }

        }
        else
        {
            ToggleLoops(); // +++ Look at comments inside this function.

            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            double z = -gamepad1.left_stick_y; // Remember, Y stick is reversed!
            double x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            double ry = gamepad1.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(ry), 1);
            double frontLeftPower = (z + x + ry) / denominator;
            double frontRightPower = (z - x - ry) / denominator;
            double backLeftPower = (z - x + ry) / denominator;
            double backRightPower = (z + x - ry) / denominator;

            frontLeftMotor.setPower(frontLeftPower * MOVEMENT_SPEED);
            frontRightMotor.setPower(frontRightPower * MOVEMENT_SPEED);
            backLeftMotor.setPower(backLeftPower * MOVEMENT_SPEED);
            backRightMotor.setPower(backRightPower * MOVEMENT_SPEED);

            // a = intake; b = intake up; y = intake down;


            if(gamepad1.b)
            {
                intakeLiftMotor.setPower(1);
                intakeLiftMotor.setTargetPosition(0); // +++ Make a Constant variable for this.
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.y)
            {
                intakeLiftMotor.setPower(1);
                intakeLiftMotor.setTargetPosition(0); // +++ Make a Constant variable for this.
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // +++ Maybe we could have something like this
            // +++ D-Pad Down: Position 0;
            // +++ D-Pad Right: Scores in Bucket: Toggle: 1st time is high bucket, 2nd time is low bucket;
            // +++ D-Pad Left: Scores Specimens: Toggle: 1st time is high bar, 2nd time is low bar;
            if(gamepad1.dpad_down)
            {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(Constants.lowBucket);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_right)
            {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(Constants.highBucket);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_left)
            {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(Constants.lowSample);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_up)
            {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(Constants.highSample);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.left_bumper)
            {
                elevatorMotor.setPower(1);
                elevatorMotor.setTargetPosition(0);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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


    public void ToggleLoops()
    {

        //============================================== ServoToggle =====================================================================

        // +++ There is a problem with this toggle. The problem is the "servoLoopBreak" is never used, but some lines that are supposed to have "servoFirstTime" now have "servoLoopBreak"
        // +++ You also don't even need the "servoLoopBreak" variable, or the "servoClosed" variable.
        if (gamepad1.x)
        {
            if (servoFirstTime == false)
            {
                servoFirstTime = true;
                if (servoFirstTime == true) // Switch "servoFirstTime" to "servoOpen"
                {
                    servoFirstTime = false; // Move this line to the "else".

                    servoOpen = true; // THis should be false, because the if checks if it is true.
                    servoLoopBreak = false; // Remove this
                }
                else
                {
                    servoClosed = true; // This should be servoOpen = true.
                    servoLoopBreak = true; // remove this line
                }
            }

        }
        else
        {
            servoLoopBreak = false; // THis should be "servoFirstTime = false;"
        }

        //==================================================== IntakeToggle ================================================

        if (gamepad1.a)
        {

            if (firstTimeIntake == false)
            {
                firstTimeIntake = true;
                if (shouldPowerIntake == true)
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

        // +++ I would recommend powering the motor outside This function, This is so you can just look for the motor powering stuff all in 1 area, but this is 100% preference. Do whatever you want.
        if (shouldPowerIntake == true)
        {
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
