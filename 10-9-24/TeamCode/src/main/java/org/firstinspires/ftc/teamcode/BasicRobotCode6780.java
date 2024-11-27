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
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.core.ColorSensorEx;
import org.firstinspires.ftc.teamcode.core.Encoder;
import org.firstinspires.ftc.teamcode.modules.DriveModule;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;

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

   
    
    
        public DcMotor frontLeftMotor = null;
    public DcMotor frontRightMotor = null;
    public DcMotor backLeftMotor = null;
    public DcMotor backRightMotor = null;

    public DcMotor elevatorMotor = null;
    private DcMotor intakeMotor;
    private DcMotor intakeLiftMotor;
    private Servo intakeServo1;
    private Servo intakeServo2;
    public Servo clawServo = null;

    
    
    private DriveModule driveModule;

    // ========================================== Intake Toggle ==========================================

    private IntakeHandler intakeHandler;
    private long lastCycleTime; // nanoSeconds
    private long currentCycleTime; // nanoSeconds
    private double deltaTime; // Seconds

    // ========================================== override control ==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride =false;

    // ========================================== Intake loop control ==========================================

    private boolean firstTimeIntake = false;
    private boolean shouldPowerIntake = false;

    // ========================================== Servo loop control ==========================================

    private boolean isClawOpen = true;
    private boolean servoFirstTime = false;



    @Override
    public void init() {

        Constants.CURRENT_TEAM = Constants.Team.Red;


        HardwareModule.GetHardware(this);


        driveModule = new DriveModule(this);
        driveModule.SetPathFindingBehavior(DriveModule.PathFindingBehavior.None);
        driveModule.SetControlBehavior(DriveModule.ControlBehavior.FirstController);


        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
    }

    @Override
    public void loop() {

        currentCycleTime = System.nanoTime();

        if (!(currentCycleTime == 0 || lastCycleTime == 0))
        {
            deltaTime = (int)(currentCycleTime - lastCycleTime) / 1000000000f;
        }

        // =============================================== Overide Toggle ===============================================
        if (gamepad2.back || gamepad1.back)
        {

            if (!isCurrentlySwitchingOverride)
            {
                isCurrentlySwitchingOverride = true;
                if (isOnOverride)
                {
                    isOnOverride = false;
                    HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    HardwareModule.intakeLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    HardwareModule.intakeLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    intakeHandler.Reset();
                    driveModule.SetControlBehavior(DriveModule.ControlBehavior.FirstController);
                }
                else
                {
                    isOnOverride = true;
                    HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    HardwareModule.intakeLiftMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                    driveModule.SetControlBehavior(DriveModule.ControlBehavior.SecondController);
                }
            }

        }
        else
        {
            isCurrentlySwitchingOverride = false;
        }



        if (isOnOverride)
        {
            /*
            // ======================================================= Drive =======================================================

            // Run wheels in tank mode (note: The joystick goes negative when pushed forward, so negate it)
            double y = -gamepad2.left_stick_y; // Remember, Y stick value is reversed
            double x = gamepad2.left_stick_x * 1.1; // Counteract imperfect strafing
            double rx = gamepad2.right_stick_x;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio,
            // but only if at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            HardwareModule.frontLeftMotor.setPower(frontLeftPower);
            HardwareModule.backLeftMotor.setPower(backLeftPower);
            HardwareModule.frontRightMotor.setPower(frontRightPower);
            HardwareModule.backRightMotor.setPower(backRightPower);
            */



            // ======================================================= INTAKE =======================================================



            if(gamepad2.dpad_left)
            {
                HardwareModule.intakeMotor.setPower(Constants.INTAKE_POWER);
                HardwareModule.intakeServo1.setPosition(1);
                HardwareModule.intakeServo2.setPosition(1);
            }
            else if(gamepad2.dpad_down)
            {
                HardwareModule.intakeMotor.setPower(-1);
                HardwareModule.intakeServo1.setPosition(-1);
                HardwareModule.intakeServo2.setPosition(-1);

            }
            else
            {
                HardwareModule.intakeMotor.setPower(0);
                HardwareModule.intakeServo1.setPosition(0.5);
                HardwareModule.intakeServo2.setPosition(0.5);
            }



            if (gamepad2.b)
            {
                HardwareModule.intakeLiftMotor.setPower(Constants.INTAKE_LIFT_POWER);
            }
            else if (gamepad2.y)
            {
                HardwareModule.intakeLiftMotor.setPower(-Constants.INTAKE_LIFT_POWER);
            }
            else
            {
                HardwareModule.intakeLiftMotor.setPower(0);
            }


            // ======================================================= Elevator =======================================================


            if(gamepad2.right_bumper)
            {
                HardwareModule.elevatorMotor.setPower(-Constants.ELEVATOR_POWER);
            }
            else if (gamepad2.right_trigger > 0.5)
            {
                HardwareModule.elevatorMotor.setPower(Constants.ELEVATOR_POWER);
            }
            else
            {
                HardwareModule.elevatorMotor.setPower(0);
            }



            // ======================================================= Claw =======================================================

            if (gamepad2.x)
            {
                HardwareModule.clawServo.setPosition(Constants.CLAW_OPEN);
            }
            else // Then the claw has to be closed
            {
                HardwareModule.clawServo.setPosition(Constants.CLAW_CLOSED);
            }
        }
        else
        {

            /*
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

            HardwareModule.frontLeftMotor.setPower(frontLeftPower * Constants.MOVEMENT_SPEED);
            HardwareModule.frontRightMotor.setPower(frontRightPower * Constants.MOVEMENT_SPEED);
            HardwareModule.backLeftMotor.setPower(backLeftPower * Constants.MOVEMENT_SPEED);
            HardwareModule.backRightMotor.setPower(backRightPower * Constants.MOVEMENT_SPEED);
            */
            // a = intake; b = intake up; y = intake down;

            ToggleLoops();


            // ======================================================= Intake =======================================================
/*
            if (shouldPowerIntake)
            {
                IntakeHandler.Action intateAction = intakeHandler.Update(deltaTime);

                if (intateAction == IntakeHandler.Action.InTake)
                {
                    clawServo.setPosition(Constants.CLAW_GRAB);
                    intakeMotor.setPower(Constants.INTAKE_POWER);
                    intakeServo1.setPosition(Constants.INTAKE_SERVO_POWER_FORWARD);
                    intakeServo2.setPosition(Constants.INTAKE_SERVO_POWER_FORWARD);
                }
                else if (intateAction == IntakeHandler.Action.OutTake)
                {
                    clawServo.setPosition(Constants.CLAW_GRAB);
                    intakeMotor.setPower(-Constants.INTAKE_POWER);
                    intakeServo1.setPosition(Constants.INTAKE_SERVO_POWER_BACKWARD);
                    intakeServo2.setPosition(Constants.INTAKE_SERVO_POWER_BACKWARD);
                }
                else if (intateAction == IntakeHandler.Action.ShutDown)
                {
                    clawServo.setPosition(Constants.CLAW_CLOSED);
                    intakeMotor.setPower(0);
                    intakeServo1.setPosition(Constants.INTAKE_SERVO_POWER_OFF);
                    intakeServo2.setPosition(Constants.INTAKE_SERVO_POWER_OFF);
                    shouldPowerIntake = false;
                }
            }
            else
            {
                intakeMotor.setPower(0);
                intakeServo1.setPosition(Constants.INTAKE_SERVO_POWER_OFF);
                intakeServo2.setPosition(Constants.INTAKE_SERVO_POWER_OFF);
            }
            */


            if(gamepad1.a)
            {
                HardwareModule.intakeMotor.setPower(Constants.INTAKE_POWER);
                HardwareModule.intakeServo1.setPosition(Constants.INTAKE_SERVO_POWER_FORWARD);
                HardwareModule.intakeServo2.setPosition(Constants.INTAKE_SERVO_POWER_FORWARD);
            }
            else
            {
                HardwareModule.intakeMotor.setPower(0);
                HardwareModule.intakeServo1.setPosition(Constants.INTAKE_SERVO_POWER_OFF);
                HardwareModule.intakeServo2.setPosition(Constants.INTAKE_SERVO_POWER_OFF);
            }


            if(gamepad1.b)
            {
                HardwareModule.intakeLiftMotor.setPower(Constants.INTAKE_LIFT_POWER);
                HardwareModule.intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_UP);
                HardwareModule.intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }

            if(gamepad1.y)
            {
                HardwareModule.intakeLiftMotor.setPower(Constants.INTAKE_LIFT_POWER);
                HardwareModule.intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_DOWN);
                HardwareModule.intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }



            // ======================================================= Elevator =======================================================

            if(gamepad1.dpad_down)
            {
                HardwareModule.elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                HardwareModule.elevatorMotor.setTargetPosition(Constants.LOW_BUCKET);
                HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_right)
            {
                HardwareModule.elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                HardwareModule.elevatorMotor.setTargetPosition(Constants.HIGH_BUCKET);
                HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_left)
            {
                HardwareModule.elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                HardwareModule.elevatorMotor.setTargetPosition(Constants.LOW_SAMPLE);
                HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.dpad_up)
            {
                HardwareModule.elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                HardwareModule.elevatorMotor.setTargetPosition(Constants.HIGH_SAMPLE);
                HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }
            if(gamepad1.left_bumper)
            {
                HardwareModule.elevatorMotor.setPower(Constants.ELEVATOR_POWER);
                HardwareModule.elevatorMotor.setTargetPosition(Constants.ELEVATOR_RETRACTED);
                HardwareModule.elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            }


            // ======================================================= Claw =======================================================


            if (isClawOpen)
            {
                intakeHandler.DropSample();
                HardwareModule.clawServo.setPosition(Constants.CLAW_OPEN);
            }
            else // THen the claw has to be closed
            {
                HardwareModule.clawServo.setPosition(Constants.CLAW_CLOSED);
            }

        }

        lastCycleTime = currentCycleTime;



        telemetry.update();
    }



    @Override
    public void stop() {
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
            servoFirstTime = false;
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

}
