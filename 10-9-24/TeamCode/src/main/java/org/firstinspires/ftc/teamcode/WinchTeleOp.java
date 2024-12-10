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

import org.firstinspires.ftc.teamcode.modules.DriveModule;
import org.firstinspires.ftc.teamcode.modules.HardwareModule;


@TeleOp(name="6780 Winch Teleop", group="Robot")
public class WinchTeleOp extends OpMode
{
    private DriveModule driveModule;

    // ========================================== override control ==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride =false;
    boolean isOpen = false;
    Boolean isClosed = false;
    @Override
    public void init() {

        // Constants.CURRENT_TEAM = Constants.Team.Red;


        HardwareModule.GetHardware(this);


        driveModule = new DriveModule(this);
        driveModule.SetPathFindingBehavior(DriveModule.PathFindingBehavior.None);
        driveModule.SetControlBehavior(DriveModule.ControlBehavior.None);


        telemetry.addData(">", "Robot Ready.  Press Play.");    //
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start()  {
        driveModule.start();
    }

    @Override
    public void loop() {
        int slidePoshion = HardwareModule.elavatorMotorRight.getCurrentPosition();
        int winchPoshion = HardwareModule.winchMotorRight.getCurrentPosition();


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



        if (isOnOverride)
        {
            Normall();

           if(slidePoshion < Constants.slideMin)
           {
               if(gamepad1.right_trigger>0.5)
               {
                   HardwareModule.winchMotorRight.setTargetPosition(Constants.winchUp);
                   HardwareModule.WinchMotorLeft.setTargetPosition(Constants.winchUp);
                   HardwareModule.WinchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                   HardwareModule.winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               }
           }

            if (slidePoshion == Constants.elavatorDown)
            {
                if(gamepad1.right_bumper)
                {
                    HardwareModule.winchMotorRight.setTargetPosition(Constants.winchDown);
                    HardwareModule.WinchMotorLeft.setTargetPosition(Constants.winchDown);
                    HardwareModule.WinchMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    HardwareModule.winchMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

            }


            if( winchPoshion == Constants.winchUp || winchPoshion == Constants.winchDown)
            {
                if (gamepad1.dpad_down) {
                    HardwareModule.elavatorMotorRight.setTargetPosition(Constants.HIGH_BUCKET);
                    HardwareModule.elavatorMotorLeft.setTargetPosition(Constants.HIGH_BUCKET);
                    HardwareModule.elavatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    HardwareModule.elavatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad1.dpad_right) {
                    HardwareModule.elavatorMotorRight.setTargetPosition(Constants.LOW_BUCKET);
                    HardwareModule.elavatorMotorLeft.setTargetPosition(Constants.LOW_BUCKET);
                    HardwareModule.elavatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    HardwareModule.elavatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


                if (gamepad1.dpad_up) {
                    HardwareModule.elavatorMotorRight.setTargetPosition(Constants.HIGH_SAMPLE);
                    HardwareModule.elavatorMotorLeft.setTargetPosition(Constants.HIGH_SAMPLE);
                    HardwareModule.elavatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    HardwareModule.elavatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }


                if (gamepad1.dpad_left) {
                    HardwareModule.elavatorMotorRight.setTargetPosition(Constants.LOW_SAMPLE);
                    HardwareModule.elavatorMotorLeft.setTargetPosition(Constants.LOW_SAMPLE);
                    HardwareModule.elavatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    HardwareModule.elavatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

                if (gamepad1.left_bumper) {
                    HardwareModule.elavatorMotorRight.setTargetPosition(Constants.elavatorDown);
                    HardwareModule.elavatorMotorLeft.setTargetPosition(Constants.elavatorDown);
                    HardwareModule.elavatorMotorRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    HardwareModule.elavatorMotorLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                }

            }


            if (gamepad1.a)
            {

                if (!isOpen)
                {
                    isOpen = true;
                    if (isClosed)
                    {
                        isClosed = false;

                    }
                    else
                    {
                        isClosed = true;


                    }
                }

            }
            else
            {
                isOpen = false;
            }
            if(isOpen == true)
            {
                HardwareModule.clawGrabbingservo.setPosition(Constants.clawOpen);
            }
           else
           {
               HardwareModule.clawGrabbingservo.setPosition(Constants.clawClosed);
           }

           if(slidePoshion == Constants.HIGH_BUCKET || slidePoshion == Constants.LOW_BUCKET)
           {
               HardwareModule.clawServoY1.setPosition(Constants.clawBucket);
               HardwareModule.clawServoY2.setPosition(Constants.clawBucket);
           }

            else if(slidePoshion == Constants.HIGH_SAMPLE || slidePoshion == Constants.LOW_SAMPLE)
            {
                HardwareModule.clawServoY1.setPosition(Constants.clawSample);
                HardwareModule.clawServoY2.setPosition(Constants.clawSample);
            }

            else if (slidePoshion == Constants.elavatorDown)
            {
                HardwareModule.clawServoY1.setPosition(Constants.clawPickUp);
                HardwareModule.clawServoY2.setPosition(Constants.clawPickUp);
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
        driveModule.Stop();
    }



    private void Normall()
    {
        // Drive Module powers the drive Code
    }
    private void Overrride()
    {
        // Drive Module powers the drive Code
    }

    public void sevoX()
    {
        double servoPoshion = gamepad1.left_trigger;
    }



}
