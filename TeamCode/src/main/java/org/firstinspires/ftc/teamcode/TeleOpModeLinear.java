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

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


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

@TeleOp(name="TeleOpMode_Linear", group="Linear Opmode")
//@Disabled
public class TeleOpModeLinear extends LinearOpMode {

    // Declare OpMode members.
    BaseRobot baseRobot = new BaseRobot();
    private ElapsedTime runtime = new ElapsedTime();

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        baseRobot.init(hardwareMap);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {


            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            double direction;
            /*
            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            double drive = -gamepad1.right_trigger;
            if(gamepad1.dpad_down)
            {

            }
            double turn  =  gamepad1.left_trigger;
            leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;

            // Send calculated power to wheels
            baseRobot.leftMotor.setPower(leftPower);
            baseRobot.rightMotor.setPower(rightPower);

            //Stop Robot
            if(gamepad1.x) {}

            if(gamepad1.b) {}

            //Arm Up
            if(gamepad1.y) { baseRobot.armMotor.setPower(0.25);}

            //Arm Down
            if(gamepad1.a) {baseRobot.armMotor.setPower(-0.25);}

            if(gamepad1.left_stick_button) {
                baseRobot.armMotor.setPower(gamepad1.left_stick_y);
            }
            */

            //move forward
            baseRobot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
            baseRobot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftPower = gamepad1.right_trigger;
            rightPower = gamepad1.right_trigger;
            baseRobot.leftMotor.setPower(leftPower);
            baseRobot.rightMotor.setPower(rightPower);

            //move backward
            baseRobot.leftMotor.setDirection(DcMotor.Direction.FORWARD);
            baseRobot.rightMotor.setDirection(DcMotor.Direction.REVERSE);
            leftPower = -gamepad1.left_trigger;
            rightPower = -gamepad1.left_trigger;
            baseRobot.leftMotor.setPower(leftPower);
            baseRobot.rightMotor.setPower(rightPower);

            //left turn
            if(gamepad1.dpad_left) {
                leftPower = 0.4;
                rightPower = -gamepad1.right_trigger;
                baseRobot.leftMotor.setPower(leftPower);
                baseRobot.rightMotor.setPower(rightPower);
            }

            //right turn
            if(gamepad1.dpad_right) {
                leftPower = -gamepad1.right_trigger;
                rightPower = 0.4;
                baseRobot.leftMotor.setPower(leftPower);
                baseRobot.rightMotor.setPower(rightPower);
            }

            baseRobot.armMotor.setDirection(DcMotor.Direction.FORWARD);
            baseRobot.armMotor.setPower(gamepad1.right_stick_y);


            baseRobot.armMotor.setDirection(DcMotor.Direction.REVERSE);
            baseRobot.armMotor.setPower(gamepad1.right_stick_y);


            //arm up
            if (gamepad1.y) {
                baseRobot.armMotor.setDirection(DcMotor.Direction.FORWARD);
                baseRobot.armMotor.setPower(0.3);
            }

            //arm down
            if (gamepad1.a) {
                baseRobot.armMotor.setDirection(DcMotor.Direction.REVERSE);
                baseRobot.armMotor.setPower(-0.2);
            }
            //stop robot
            if(gamepad1.start)
            {
                baseRobot.StopRobot(null);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.update();
        }
    }
}
