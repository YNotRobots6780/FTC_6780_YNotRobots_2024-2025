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

@Autonomous(name="Odomatry Auto", group="Robot")
// @Disabled
public class SampleAuto extends LinearOpMode 
{

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
    public boleian in line = false

    @Override
    public void runOpMode() 
    {

        Constants.CURRENT_TEAM = Constants.Team.Red;


        HardwareModule.GetHardware(this);


        telemetry.addData(">", "Ready to Run");
        waitForStart();


               
      
     public void pathFindeing()
     {

        telemetry.addData(">", Constants.F_B)
        telemetry.addData(">", Constants.L_R);
       
       
        int frontRight = frontRightMotor.getCurrentPosition();
        int frontLeft = frontLeftMotor.getCurrentPosition();
        int back = intakeMotor.getCurrentPosition();
        int targetPoshion_F_B = 1000;
        int targetPoshion_L_R = 1000;
        boleian isRotating = false;
        int slowingMathLR = targetPoshion_L_R - Constants.L_R;
        int slowingMathFB = targetPoshion_F_B - Constants.F_B;
        int slowingMathNegativeLR = targetPoshion_L_R + Constants.L_R;
        int slowingMathNegativeFB = targetPoshion_F_B + Constants.F_B;
        
       
        else if (Constants.L_R > targetPoshion_L_R) 
        {
            // Shimmy Right
            frontRightMotor.setPower(x + 0.5);
            backRightMotor.setPower(x + -0.5);
            frontLeftMotor.setPower(x + -0.5);
            backLeftMotor.setPower(x + -0.5);        
        }

        
        
        

        else if (Constants.L_R < targetPoshion_L_R) 
        {
            //shimmy Left
            frontRightMotor.setPower(x + 0.5);
            backRightMotor.setPower(X + -0.5);
            frontLeftMotor.setPower(x + 0.5);
            backLeftMotor.setPower(x + -0.5);
        }


        

       else if (Constants.L_R == targetPoshion_L_R) 
        {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        }

        else if (Constants.F_B == targetPoshion_F_B) 
        {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        
        
        }

       
        else if (Constants.F_B > targetPoshion_F_B)
        {
            backLeftMotor.setPower(x + -0.5);
            frontLeftMotor.setPower(x + -0.5);
            backRightMotor.setPower(x + -0.5);
            frontRightMotor.setPower(x + -0.5);
        }
        
        else i (Constants.F_B < targetPoshion_F_B) 
        {
            backLeftMotor.setPower(X + 0.5);
            frontLeftMotor.setPower(x + 0.5);
            backRightMotor.setPower(x + 0.5);
            frontRightMotor.setPower(x + 0.5);
        }


    




        public void x ()
        {
          
          if else (Constants.R == 180) 
          {
            char x = "-"
          } 
        
          
            if (Constants.R == 0)
           {
            char x = "0"
           } 
        
        
        }



            public void rotate (string deirection;) 
            {
                else if (Constants.currentPoshion == Constants.sample1)
                {
                    
                    
                    else if (rotashion > sa1)
                    {
                    backLeftMotor.setPower( 0.5);
                    frontLeftMotor.setPower( 0.5);
                    backRightMotor.setPower(- 0.5);
                    frontRightMotor.setPower(- 0.5);
                    }
                    else if (rotashion < sa1)
                    {
                    backLeftMotor.setPower( -0.5);
                    frontLeftMotor.setPower( -0.5);
                    backRightMotor.setPower( 0.5);
                    frontRightMotor.setPower( 0.5);
                    }
                    
                    else
                    {
                        
                    }
                
                }
           
           
                else if( Constants.sample2 == Constants.currentPoshion)
                {
               
                    //pick up sample
                backLeftMotor.setPower(0);
                frontLeftMotor.setPower( 0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                }
            
               else if ( Constants.sub1 == Constants.currentPoshion)
                {
                   else if(rotashion > su1)
                   {
                    backLeftMotor.setPower( 0.5);
                    frontLeftMotor.setPower( 0.5);
                    backRightMotor.setPower(- 0.5);
                    frontRightMotor.setPower(- 0.5);
                   }
                   
                   else if (rotashion < su1)
                   {
                    backLeftMotor.setPower( -0.5);
                    frontLeftMotor.setPower( -0.5);
                    backRightMotor.setPower( 0.5);
                    frontRightMotor.setPower( 0.5);
                   }
                    else
                    {

                    }
                
                
                }
           
               else if( Constants.sub2, == Constants.currentPoshion )
                {
                 backLeftMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                }

            }



            public void rotashion ()
            {
             int rotashion = Constants.L_R - rotashionPart1;
           
             if (isRotating == false)
             {
                int rotashionPart1 = F_B;
             }
                  
             else if (Constants.currentPoshion == Constants.sample1)
             {
                isRotating = true;
             }
            
             else if (Constants.currentPoshion == Constants.sub1)
             {
                isRotating = false;
             }
        
             else
             {
                isRotating = false;
             }
            
            
            
            
            }
    
    
    
    
        }


    }
}/* Copyright (c) 2017 FIRST. All rights reserved.
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

@Autonomous(name="Odomatry Auto", group="Robot")
// @Disabled
public class SampleAuto extends LinearOpMode 
{

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
    public boleian in line = false

    @Override
    public void runOpMode() 
    {

        Constants.CURRENT_TEAM = Constants.Team.Red;


        HardwareModule.GetHardware(this);


        telemetry.addData(">", "Ready to Run");
        waitForStart();


               
      
     public void pathFindeing()
     {

        telemetry.addData(">", Constants.F_B)
        telemetry.addData(">", Constants.L_R);
       
       
        int frontRight = frontRightMotor.getCurrentPosition();
        int frontLeft = frontLeftMotor.getCurrentPosition();
        int back = intakeMotor.getCurrentPosition();
        int targetPoshion_F_B = 1000;
        int targetPoshion_L_R = 1000;
        boleian isRotating = false;
        int slowingMathLR = targetPoshion_L_R - Constants.L_R;
        int slowingMathFB = targetPoshion_F_B - Constants.F_B;
        int slowingMathNegativeLR = targetPoshion_L_R + Constants.L_R;
        int slowingMathNegativeFB = targetPoshion_F_B + Constants.F_B;
        
       
        else if (Constants.L_R > targetPoshion_L_R) 
        {
            // Shimmy Right
            frontRightMotor.setPower(x + 0.5);
            backRightMotor.setPower(x + -0.5);
            frontLeftMotor.setPower(x + -0.5);
            backLeftMotor.setPower(x + -0.5);        
        }

        
        
        

        else if (Constants.L_R < targetPoshion_L_R) 
        {
            //shimmy Left
            frontRightMotor.setPower(x + 0.5);
            backRightMotor.setPower(X + -0.5);
            frontLeftMotor.setPower(x + 0.5);
            backLeftMotor.setPower(x + -0.5);
        }


        

       else if (Constants.L_R == targetPoshion_L_R) 
        {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        }

        else if (Constants.F_B == targetPoshion_F_B) 
        {
            frontRightMotor.setPower(0);
            backRightMotor.setPower(0);
            frontLeftMotor.setPower(0);
            backLeftMotor.setPower(0);
        
        
        }

       
        else if (Constants.F_B > targetPoshion_F_B)
        {
            backLeftMotor.setPower(x + -0.5);
            frontLeftMotor.setPower(x + -0.5);
            backRightMotor.setPower(x + -0.5);
            frontRightMotor.setPower(x + -0.5);
        }
        
        else i (Constants.F_B < targetPoshion_F_B) 
        {
            backLeftMotor.setPower(X + 0.5);
            frontLeftMotor.setPower(x + 0.5);
            backRightMotor.setPower(x + 0.5);
            frontRightMotor.setPower(x + 0.5);
        }


    




        public void x ()
        {
          
          if else (Constants.R == 180) 
          {
            char x = "-"
          } 
        
          
            if (Constants.R == 0)
           {
            char x = "0"
           } 
        
        
        }



            public void rotate (string deirection;) 
            {
                else if (Constants.currentPoshion == Constants.sample1)
                {
                    
                    
                    else if (rotashion > sa1)
                    {
                    backLeftMotor.setPower( 0.5);
                    frontLeftMotor.setPower( 0.5);
                    backRightMotor.setPower(- 0.5);
                    frontRightMotor.setPower(- 0.5);
                    }
                    else if (rotashion < sa1)
                    {
                    backLeftMotor.setPower( -0.5);
                    frontLeftMotor.setPower( -0.5);
                    backRightMotor.setPower( 0.5);
                    frontRightMotor.setPower( 0.5);
                    }
                    
                    else
                    {
                        
                    }
                
                }
           
           
                else if( Constants.sample2 == Constants.currentPoshion)
                {
               
                    //pick up sample
                backLeftMotor.setPower(0);
                frontLeftMotor.setPower( 0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                }
            
               else if ( Constants.sub1 == Constants.currentPoshion)
                {
                   else if(rotashion > su1)
                   {
                    backLeftMotor.setPower( 0.5);
                    frontLeftMotor.setPower( 0.5);
                    backRightMotor.setPower(- 0.5);
                    frontRightMotor.setPower(- 0.5);
                   }
                   
                   else if (rotashion < su1)
                   {
                    backLeftMotor.setPower( -0.5);
                    frontLeftMotor.setPower( -0.5);
                    backRightMotor.setPower( 0.5);
                    frontRightMotor.setPower( 0.5);
                   }
                    else
                    {

                    }
                
                
                }
           
               else if( Constants.sub2, == Constants.currentPoshion )
                {
                 backLeftMotor.setPower(0);
                frontLeftMotor.setPower(0);
                backRightMotor.setPower(0);
                frontRightMotor.setPower(0);
                }

            }



            public void rotashion ()
            {
             int rotashion = Constants.L_R - rotashionPart1;
           
             if (isRotating == false)
             {
                int rotashionPart1 = F_B;
             }
                  
             else if (Constants.currentPoshion == Constants.sample1)
             {
                isRotating = true;
             }
            
             else if (Constants.currentPoshion == Constants.sub1)
             {
                isRotating = false;
             }
        
             else
             {
                isRotating = false;
             }
            
            
            
            
            }
    
    
    
    
        }


    }
}
