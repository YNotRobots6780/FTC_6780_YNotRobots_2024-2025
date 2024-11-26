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
import com.qualcomm.robotcore.exception.TargetPositionNotSetException;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.core.ColorSensorEx;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;

import java.lang.annotation.Target;

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

@Autonomous(name="Sample Auto", group="Robot")
// @Disabled
public class SampleAuto extends LinearOpMode {

    /* Declare OpMode members. */
    private ElapsedTime runtime = new ElapsedTime();
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


    private ColorSensorEx frontIntakeColorSensor;
    int driveTime = 0;


    // Elevator
    public int targetElevatorPosition;


    @Override
    public void runOpMode() {

        Constants.CURRENT_TEAM = Constants.Team.Red;


        HardwareModule.GetHardware(this);


        telemetry.addData(">", "Ready to Run");


        waitForStart();

    /*
       elevatorMotor.setPower(1);
        elevatorMotor.setTargetPosition(Constants.HIGH_BUCKET);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
          // enter servo=============================================================================

           sleep(6000);
            Return();
            sleep(8000);
            shimmyRight(4000);
            sleep(driveTime);
            turnRight(500);
            sleep(driveTime);
            MoveRobotFoward(4000);

        */
      /*  intakeLiftMotor.setPower(1);
        intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_UP);
        intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        sleep(2000);
        MoveRobotFoward(500);
        sleep(1000);
        Out(Constants.HIGH_BUCKET);
        sleep(2000);
       clawServo.setPosition(Constants.CLAW_OPEN);
       sleep(5000);
       clawServo.setPosition(Constants.CLAW_CLOSED);
       MoveRobotBack(500);
        Return();
        sleep(2000);
        shimmyLeft(100);
        pickUp(4000,500, true);
*/
        //pathFiding();
        //sleep(30000);


        intakeLiftMotor.setPower(1);
        intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_UP);
        intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
     sleep(4000);
    Out(Constants.HIGH_BUCKET);
    sleep(30000);


    }


    private void Return()
    {
        elevatorMotor.setTargetPosition(0);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }


    private void MoveRobotFoward(int driveTime) {
        backLeftMotor.setPower(0.5);
        frontLeftMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        frontRightMotor.setPower(0.5);
        sleep(driveTime);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    private void turnLeft(int driveTime) {
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(0.5);
        frontLeftMotor.setPower(-0.5);
        backLeftMotor.setPower(-0.5);
        sleep(driveTime);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    private void turnRight(int driveTime) {
        frontRightMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);
        frontLeftMotor.setPower(0.5);
        backLeftMotor.setPower(0.5);
        sleep(driveTime);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    private void shimmyRight(int driveTime) {
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(-0.5);
        frontLeftMotor.setPower(0.5);
        backLeftMotor.setPower(-0.5);
        sleep(driveTime);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }

    private void shimmyLeft(int driveTime) {
        frontRightMotor.setPower(0.5);
        backRightMotor.setPower(-0.5);
        frontLeftMotor.setPower(-0.5);
        backLeftMotor.setPower(0.5);
        sleep(driveTime);
        frontRightMotor.setPower(0);
        backRightMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backLeftMotor.setPower(0);
    }


    public void MoveRobot(double x, double z, double rotation) {   // Denominator is the largest motor power (absolute value) or 1
        // This ensures all the powers maintain the same ratio, but only when
        // at least one is out of the range [-1, 1]
        double denominator = Math.max(Math.abs(z) + Math.abs(x) + Math.abs(rotation), 1);
        double frontLeftPower = (z + x + rotation) / denominator;
        double frontRightPower = (z - x - rotation) / denominator;
        double backLeftPower = (z - x + rotation) / denominator;
        double backRightPower = (z + x - rotation) / denominator;

        frontLeftMotor.setPower(frontLeftPower);
        frontRightMotor.setPower(frontRightPower);
        backLeftMotor.setPower(backLeftPower);
        backRightMotor.setPower(backRightPower);
    }

    private void MoveRobotBack(int driveTime) {
        backLeftMotor.setPower(-0.5);
        frontLeftMotor.setPower(-0.5);
        backRightMotor.setPower(-0.5);
        frontRightMotor.setPower(-0.5);
        sleep(driveTime);
        backLeftMotor.setPower(0);
        frontLeftMotor.setPower(0);
        backRightMotor.setPower(0);
        frontRightMotor.setPower(0);
    }

    private void intake(int runTime) {
        intakeMotor.setPower(Constants.INTAKE_POWER);
        intakeServo1.setPosition(1);
        intakeServo2.setPosition(1);
        sleep(runTime);
        intakeMotor.setPower(0);
        intakeServo1.setPosition(0);
        intakeServo2.setPosition(0);
    }


    private void Out(int targetPosition) {
        elevatorMotor.setPower(Constants.ELEVATOR_POWER);
        elevatorMotor.setTargetPosition(targetPosition);
        elevatorMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void pickUp(int driveTime, int driveDistece, boolean runing) {
        if (runing == true) {
            intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_DOWN);
            intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(Constants.INTAKE_POWER);
            intakeServo1.setPosition(1);
            intakeServo2.setPosition(1);
            sleep(driveTime);
            intakeLiftMotor.setTargetPosition(Constants.INTAKE_LIFT_UP);
            intakeLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            intakeMotor.setPower(0);
            intakeServo1.setPosition(0);
            intakeServo2.setPosition(0);

        }
        if (runing == true) {
            backLeftMotor.setPower(0.5);
            frontLeftMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
            sleep(driveDistece);
            backLeftMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backRightMotor.setPower(0);
            frontRightMotor.setPower(0);
        }

    }
//
    /////////////////////////////////////////////////////////////////////////////////////////////////////////
    //////////////////////////////////////////////////////////////////////////////////////////////////////
    ///////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////////


    public void pathFiding() {

        int frontRight = frontRightMotor.getCurrentPosition();
        int frontLeft = frontLeftMotor.getCurrentPosition();
        int back = intakeMotor.getCurrentPosition();
        int F_B = frontRight + frontLeft / 2;
        int L_R = back;
        int targetPoshion_F_B = 1000;
        int targetPoshion_L_R = 1000;

        if (L_R > targetPoshion_L_R) {
            // Shimmy Right
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(-0.5);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(-0.5);
        }

        sleep(1000);

        if (L_R < targetPoshion_L_R) {
            //shimmy Left
            frontRightMotor.setPower(0.5);
            backRightMotor.setPower(-0.5);
            frontLeftMotor.setPower(0.5);
            backLeftMotor.setPower(-0.5);
        }


        sleep(1000);

        if (L_R == targetPoshion_L_R) {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        }

        sleep(1000);

        if (F_B > targetPoshion_F_B) {
            backLeftMotor.setPower(-0.5);
            frontLeftMotor.setPower(-0.5);
            backRightMotor.setPower(-0.5);
            frontRightMotor.setPower(-0.5);
        }

        sleep(1000);

        if (F_B < targetPoshion_F_B) {
            backLeftMotor.setPower(0.5);
            frontLeftMotor.setPower(0.5);
            backRightMotor.setPower(0.5);
            frontRightMotor.setPower(0.5);
        }


    }


}