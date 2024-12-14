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

@TeleOp(name="6780 Interview Teleop", group="Robot")
public class WinchTeleOpInterview extends OpMode
{
// ========================================== override control ==========================================

    private boolean isOnOverride = false;
    private boolean isCurrentlySwitchingOverride =false;


    @Override
    public void init() {

        // Constants.CURRENT_TEAM = Constants.Team.Red;


        HardwareModule.GetHardware(hardwareMap);


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
                if (isOnOverride)
                {
                    isOnOverride = false;
                }
                else
                {
                    isOnOverride = true;
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
        // Drive Module powers the drive Code
    }
    private void Overrride()
    {
        // Drive Module powers the drive Code
    }

}
