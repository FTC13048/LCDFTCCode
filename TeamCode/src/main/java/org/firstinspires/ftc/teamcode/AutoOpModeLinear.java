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
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import junit.framework.Test;

@Autonomous(name="AutoOpMode_Linear", group="Linear Opmode")
//@Disabled
public class AutoOpModeLinear extends LinearOpMode {

    BaseRobot baseRobot = new BaseRobot();

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private static final double INCREMENT   = 0.01;     // amount to slew servo each CYCLE_MS cycle
    private static final int    CYCLE_MS    =   50;     // period of each cycle
    private static final double MAX_POS     =  1.0;     // Maximum rotational position
    private static final double MIN_POS     =  0.0;     // Minimum rotational position

    public DcMotor  leftMotor  = null;
    public DcMotor  rightMotor  = null;
    public DcMotor  armMotor = null;
    public DcMotor  sweeperMotor     = null;
    public Servo armServo = null;

    private double  position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public static final double MID_SERVO       =  0.5 ;
    public static final double ARM_UP_POWER    =  0.45 ;
    public static final double ARM_DOWN_POWER  = -0.45 ;
    private boolean rampUp = true;

    @Override
    public void runOpMode() {
        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        baseRobot.init(hardwareMap);

        // Wait for the start button
        telemetry.addData(">", "Press Start to scan Servo." );
        telemetry.update();
        waitForStart();
        runtime.reset();


        // Run code till stop pressed.
        while(opModeIsActive()){
            //baseRobot.DriveRobot(.5, .5, 1000);
           baseRobot.lowerLatch();
           baseRobot.DriveRobot(.5, .5, 1000);
           baseRobot.latchMotor.setPower(0);
          //  StopAllMotors();
           //lower robot
           // baseRobot.LiftBasket();
            //baseRobot.DropBasket();
            /*  baseRobot.DriveRobot(.5, 0, 1000);                      //face the crater/depot

                    //PICK ONE OF THE BELOW 6 -- YOU ARE  T H E   R O B O T

                    //CRATER LEFT
                    baseRobot.DriveRobot(0, .2, 250);               //turn left to face the mineral
                    baseRobot.DriveRobot(.25, .25, 1000);           //ram gold mineral
                    baseRobot.DriveRobot(-
                    +5, -.5, 1000);           //Back away from minerals

                    //CRATER RIGHT
                    baseRobot.DriveRobot(.2, 0, 250);               //turn right to face the mineral
                    baseRobot.DriveRobot(.25, .25, 1000);           //ram gold mineral
                    baseRobot.DriveRobot(-.5, -.5, 1000);           //Back away from minerals

                    //CRATER CENTER
                    baseRobot.DriveRobot(.25, .25, 1250);           //ram gold mineral
                    baseRobot.DriveRobot(-.5, -.5, 1000 );          //Back away from minerals

                    //DEPOT LEFT
                    baseRobot.DriveRobot(0, .2, 250);               //turn left to face the mineral
                    baseRobot.DriveRobot(.25, .25, 1000);           //ram gold mineral
                    baseRobot.DriveRobot(-.5, -.5, 1000);           //Back away from minerals

                    //DEPOT RIGHT
                    baseRobot.DriveRobot(.2, 0, 250);               //turn right to face the mineral
                    baseRobot.DriveRobot(.25, .25, 1000);           //ram gold mineral
                    baseRobot.DriveRobot(-.5, -.5, 1000);           //Back away from minerals

                    //DEPOT CENTER
                    baseRobot.DriveRobot(.25, .25, 1000);           //Ram gold mineral
                    baseRobot.DriveRobot(.25, .25, 1000);           //move a little extra into the depot
*/
            telemetry.addData("Status", "Run Time: " + runtime.seconds());
            telemetry.update();
            /*
            if(runtime.seconds() < 3) {
                LiftBasket();
            }
            else if(runtime.seconds() < 6)
            {
                DropBasket();
                StopAllMotors();
            }
            */
            //LiftBasket();
        }

        // Signal done;
        telemetry.addData(">", "Done");
        telemetry.update();
    }

    private void TestServo()
    {
        // slew the servo, according to the rampUp (direction) variable.
        if (rampUp) {
            // Keep stepping up until we hit the max value.
            position += INCREMENT ;
            if (position >= MAX_POS ) {
                position = MAX_POS;
                rampUp = !rampUp;   // Switch ramp direction
            }
        }
        else {
            // Keep stepping down until we hit the min value.
            position -= INCREMENT ;
            if (position <= MIN_POS ) {
                position = MIN_POS;
                rampUp = !rampUp;  // Switch ramp direction

            }
        }

        // Display the current value
        telemetry.addData("Servo Position", "%5.2f", position);
        telemetry.addData(">", "Press Stop to end test." );
        telemetry.update();

        // Set the servo to the new position and pause;
        baseRobot.armServo.setPosition(position);
        sleep(CYCLE_MS);
        idle();
    }

    private void DriveMotor()
    {
        // Landing the root
        //baseRobot.armMotor.setDirection(DcMotor.Direction.REVERSE);
        //baseRobot.armMotor.setPower(0.15);

        //Move the robot
        /*
        baseRobot.leftMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        baseRobot.leftMotor.setPower(0.25);
        baseRobot.rightMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        baseRobot.rightMotor.setPower(0.25);
        */
        baseRobot.armMotor.setDirection(DcMotor.Direction.FORWARD);// Set to FORWARD if using AndyMark motors
        baseRobot.armMotor.setPower(0.20);
    }

    private void StopAllMotors()
    {
        // Set all motors to zero power
        baseRobot.leftMotor.setPower(0);
        baseRobot.rightMotor.setPower(0);
        baseRobot.armMotor.setPower(0);
    }

    private void LiftBasket()
    {
        //baseRobot.armServo.setPosition(-position);
        baseRobot.armMotor.setDirection(DcMotor.Direction.FORWARD);
        baseRobot.armMotor.setPower(0.40);
        //sleep(CYCLE_MS);
        //idle();
    }

    private void DropBasket()
    {
        //baseRobot.armServo.setPosition(-position);
        baseRobot.armMotor.setDirection(DcMotor.Direction.REVERSE);
        baseRobot.armMotor.setPower(-0.35);
        sleep(CYCLE_MS);
        idle();
    }
}
