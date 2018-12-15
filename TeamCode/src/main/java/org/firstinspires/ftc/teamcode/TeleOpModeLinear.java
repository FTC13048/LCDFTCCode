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
    FTCBaseRobot baseRobot = new FTCBaseRobot();
    private ElapsedTime runtime = new ElapsedTime();

    // Setup a variable for each drive wheel to save power level for telemetry
    double leftPower =0;
    double rightPower =0;
    double armPower =0;
    double latchPower=0;
    double servoPower = 0;
    double armPosition=0;
    double direction;

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
//*************************************************************************************************
//          GAMEPAD 1
//          1. Left and Right Trigger: Gives power to the drive motors
//          2. Dpad left and right buttons: Will turn the robot left or right
//          3. Right joy stick up: Ascend the robot on the pod
//          4. Right joy stick down: Descends the robot grom the pod
//          5. Right joy stick press: Will stop the latch motor
//          6. Left joy stick: NOT USED
//          7. X,Y,A,B: NOT USED
//*************************************************************************************************

            //move forward
            leftPower = gamepad1.right_trigger;
            rightPower = gamepad1.right_trigger;
            baseRobot.DriveRobot(leftPower,rightPower);

            //move backward
            leftPower = -gamepad1.left_trigger;
            rightPower = -gamepad1.left_trigger;
            baseRobot.DriveRobot(leftPower,rightPower);

            //left turn
            if(gamepad1.dpad_left) {
                leftPower = 0.0; //0.5;
                rightPower = 0.5; //-gamepad1.right_trigger;
                baseRobot.DriveRobot(leftPower,rightPower);
            }

            //right turn
            if(gamepad1.dpad_right) {
                leftPower = 0.5; //-gamepad1.right_trigger;
                rightPower = 0.0; //0.5;
                baseRobot.DriveRobot(leftPower,rightPower);
            }

            if(gamepad1.dpad_up) {
                leftPower = -0.4;
                rightPower = -gamepad1.right_trigger;
                baseRobot.DriveRobot(leftPower,rightPower);
            }

            //right turn
            if(gamepad1.dpad_down) {
                leftPower = -gamepad1.right_trigger;
                rightPower = -0.4;
                baseRobot.DriveRobot(leftPower,rightPower);
            }

            //Latch Up and Down the Pod
            if (gamepad1.right_stick_y > 0)
            {
                latchPower = gamepad1.right_stick_y / 2.5;
                baseRobot.RobotAscend(latchPower);
            }
            else if (gamepad1.right_stick_y < 0)
            {
                latchPower = -gamepad1.right_stick_y / 2.5;
                baseRobot.RobotDescend(latchPower);
            }
            else if(gamepad1.right_stick_button)
            {
                latchPower = 0;
                baseRobot.RobotDescend(latchPower);
            }
//*************************************************************************************************
//          GAMEPAD 2
//          1. Left joy stick up or down: Will pull the arm up or down
//          2. Left joy stick press: Will stop the arm motor
//          3. Right joy stick up or down: Will extend or backtrack the basket
//          4. Right joy stick press: Will stop the basket servo
//          5. Left and Right Triggers: NOT USED
//          6. Left and Right Bumpers: NOT USED
//          7. X,Y,A,B: NOT USED
//          8. DPADS: NOT USED
//*************************************************************************************************

            //AMR Up or Down
            if (gamepad2.left_stick_y !=0)
            {
                armPower = -gamepad2.left_stick_y / 2.5;
                baseRobot.armMotor.setPower(armPower);
            }
            if(gamepad2.left_stick_button)
            {
                armPower = 0;
                baseRobot.armMotor.setPower(armPower);
            }

            //Basket Extend and Backtrack - Option 1
            if(gamepad2.right_stick_y !=0)
            {
                servoPower = -gamepad2.right_stick_y / 8;
                baseRobot.MoveBasket(servoPower);
            }

            if(gamepad2.right_stick_button)
            {
                baseRobot.MoveBasket(FTCBaseRobot.ServoPosition.STOP);
            }

//*************************************************************************************************
//          Update Driver Station with telemetry data
//*************************************************************************************************
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("ArmServo Pos", "Pos: (%.2f)", servoPower);
            telemetry.addData("ArmPower", "Pos: (%.2f)", baseRobot.armMotor.getPower());
            telemetry.update();
        }
    }
}
