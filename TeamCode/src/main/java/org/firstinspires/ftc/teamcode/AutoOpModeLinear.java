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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import junit.framework.Test;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

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


        // Run code till stop pressed.
        while(opModeIsActive()){

            //TestServo();
            DriveMotor();
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
        // Spin the sweep motor
        baseRobot.sweeperMotor.setDirection(DcMotor.Direction.REVERSE);// Set to FORWARD if using AndyMark motors
        baseRobot.sweeperMotor.setPower(0.5);

        // Landing the root
        baseRobot.armMotor.setDirection(DcMotor.Direction.REVERSE);
        baseRobot.sweeperMotor.setPower(0.15);

        // Set all motors to run without encoders.
        // May want to use RUN_USING_ENCODERS if encoders are installed.
        baseRobot.sweeperMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }
}
