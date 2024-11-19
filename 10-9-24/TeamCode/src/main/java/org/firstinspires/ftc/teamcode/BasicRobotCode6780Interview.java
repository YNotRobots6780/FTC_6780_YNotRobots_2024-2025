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

import org.firstinspires.ftc.teamcode.core.ColorSensorEx;
import org.firstinspires.ftc.teamcode.core.Encoder;

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

@TeleOp(name="6780 New robot code Interview", group="Robot")
public class BasicRobotCode6780Interview extends OpMode
{

    //=====================override control==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride =false;

    //====================Intake loop control==================================================================

    private boolean firstTimeIntake = false;
    private boolean shouldPowerIntake = false;

    //==============Servo loop control========================================================================

    private boolean isClawOpen = false;
    private boolean servoFirstTime = false;


    /* Declare OpMode members. */
    private Encoder rightOdometer;
    private Encoder leftOdometer;
    private Encoder backOdometer;

    private DcMotor intakeMotor;
    private DcMotor intakeLiftMotor;
    private DcMotor elevatorMotor;

    private Servo clawServo;
    private ColorSensorEx frontIntakeColorSensor;

    /*
     * Code to run ONCE when the driver hits INIT
     */
    @Override
    public void init() {
        // Define and Initialize Motors
        intakeMotor = hardwareMap.get(DcMotor.class, "intake_motor"); // 2
        elevatorMotor = hardwareMap.get(DcMotor.class, "elavatorMotor"); //ex 1
        intakeLiftMotor = hardwareMap.get(DcMotor.class, "intakeLiftMotor"); // 3
        clawServo = hardwareMap.get(Servo.class,"claw"); // NONE
        frontIntakeColorSensor = new ColorSensorEx(hardwareMap.get(ColorSensor.class, "frontColorSensor")); // EX: 12C 3

        leftOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        rightOdometer = new Encoder(hardwareMap.get(DcMotor.class, "front_left"));
        backOdometer = new Encoder(hardwareMap.get(DcMotor.class, "back_left"));



        elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        elevatorMotor.setDirection(DcMotorSimple.Direction.REVERSE);
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

        // =============================================== Overide Toggle ===============================================
        if (gamepad2.back || gamepad1.back)
        {

            if (!isCurrentlySwitchingOverride)
            {
                isCurrentlySwitchingOverride = true;
                if (isOnOverride)
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



        if (isOnOverride)
        {
            // ======================================================= INTAKE =======================================================

            if(gamepad2.a)
            {
                intakeMotor.setPower(Constants.INTAKE_POWER);
            }
            else {
                intakeMotor.setPower(0);
            }


            if (gamepad2.b)
            {
                intakeLiftMotor.setPower(Constants.INTAKE_LIFT_POWER);
            }
            else if (gamepad2.y)
            {
                intakeLiftMotor.setPower(-Constants.INTAKE_LIFT_POWER);
            }
            else
            {
                intakeLiftMotor.setPower(0);
            }


            // ======================================================= Elevator =======================================================


            if(gamepad2.right_bumper)
            {
                elevatorMotor.setPower(-Constants.ELEVATOR_POWER);
            }
            else if (gamepad2.right_trigger > 0.5)
            {
                elevatorMotor.setPower(Constants.ELEVATOR_POWER);
            }
            else
            {
                elevatorMotor.setPower(0);
            }



            // ======================================================= Claw =======================================================

            if (isClawOpen)
            {
                clawServo.setPosition(Constants.CLAW_OPEN);
            }
            else // THen the claw has to be closed
            {
                clawServo.setPosition(Constants.CLAW_CLOSED);
            }
        }
        else
        {
            // a = intake; b = intake up; y = intake down;

            ToggleLoops();


            // ======================================================= Intake =======================================================

            if (shouldPowerIntake)
            {
                intakeMotor.setPower(Constants.INTAKE_POWER);
            }
            else
            {
                intakeMotor.setPower(0);
            }

            if(gamepad1.b)
            {
                intakeLiftMotor.setPower(Constants.INTAKE_LIFT_POWER);
                intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_UP);
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.y)
            {
                intakeLiftMotor.setPower(Constants.INTAKE_LIFT_POWER);
                intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_DOWN);
                intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            // ======================================================= Elevator =======================================================

            if(gamepad1.dpad_down)
            {
                elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                elevatorMotor.setTargetPosition(Constants.LOW_BUCKET);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_right)
            {
                elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                elevatorMotor.setTargetPosition(Constants.HIGH_BUCKET);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_left)
            {
                elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                elevatorMotor.setTargetPosition(Constants.LOW_SAMPLE);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_up)
            {
                elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                elevatorMotor.setTargetPosition(Constants.HIGH_SAMPLE);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.left_bumper)
            {
                elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                elevatorMotor.setTargetPosition(Constants.ELEVATOR_RETRACTED);
                elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // ======================================================= Claw =======================================================


            if (isClawOpen)
            {
                clawServo.setPosition(Constants.CLAW_OPEN);
            }
            else // THen the claw has to be closed
            {
                clawServo.setPosition(Constants.CLAW_CLOSED);
            }

        }

    }

    public void ToggleLoops()
    {
        //============================================== ServoToggle =====================================================================

        if (gamepad1.x)
        {
            if (!servoFirstTime)
            {
                servoFirstTime = true;
                if (isClawOpen)
                {
                    isClawOpen = false;
                }
                else
                {
                    isClawOpen = true;
                }
            }

        }
        else
        {
            servoFirstTime = false; // Move this line to the "else".
        }

        //==================================================== IntakeToggle ================================================

        if (gamepad1.a)
        {
            if (!firstTimeIntake)
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

    }

    /*
     * Code to run ONCE after the driver hits STOP
     */
    @Override
    public void stop() {
    }
}
