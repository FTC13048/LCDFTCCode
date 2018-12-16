package org.firstinspires.ftc.teamcode;

import android.graphics.Color;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.mmPerInch;

@Autonomous(name = "AutoOpModeLinear LCD", group = "Auto Linear Opmode")
//@Disabled
public class AutoOpModeLinearLCD extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private static final double INCREMENT = 0.01;     // amount to slew servo each CYCLE_MS cycle
    private static final int CYCLE_MS = 50;     // period of each cycle
    private static final double MAX_POS = 1.0;     // Maximum rotational position
    private static final double MIN_POS = 0.0;     // Minimum rotational position

    private double position = (MAX_POS - MIN_POS) / 2; // Start at halfway position
    public static final double MID_SERVO = 0.5;
    public static final double ARM_UP_POWER = 0.45;
    public static final double ARM_DOWN_POWER = -0.45;
    private boolean rampUp = true;
    private int goldPos = 0;

    FTCBaseRobot baseRobot = new FTCBaseRobot();
    vuforiaNavRecognize vuforiaNR = new vuforiaNavRecognize();
    SensorColorLCD senseMineralColor = new SensorColorLCD();

    @Override
    public void runOpMode() {
        baseRobot.init(hardwareMap);
        vuforiaNR.initVuforia(hardwareMap);
        senseMineralColor.init(hardwareMap);

        telemetry.addData(">", "Press Start.");
        telemetry.update();
        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            //Step1: Recognize Gold
            goldPos = vuforiaNR.findGold();
            telemetry.addData("Gold Position", goldPos);
            telemetry.update();

            //Step2: Descend
            baseRobot.RobotDescend();
            sleep(2500);
            baseRobot.StopRobot(baseRobot.latchMotor);

			//Step3: Look for Gold one more time in case the last attempt failed
			if ( goldPos == 0 ) {
                vuforiaNR.setGoldFound(false);
                goldPos = vuforiaNR.findGold();
                telemetry.addData("Gold Position Again", goldPos);
                telemetry.update();
			}
            //Step4: Go towards Gold and Knock off Gold
			if ( goldPos == 1 ) {
				baseRobot.DriveRobot(0.0,0.25); //left turn
				sleep(1500);
			} else if ( goldPos == 2 ) {
				baseRobot.DriveRobot(0.4,0.4); //go straight
				sleep(2000);
			} else if ( goldPos == 3 ) {
				baseRobot.DriveRobot(0.20,0.0); //right turn
				sleep(1000);
			}

			//baseRobot.StopRobot();

             stop();

            /**Step4: See if there is a Crater ahead of you using color distance sensor

            //Step 4.1: OPTIONAL: Depending on testing, we might have to turn the robot a little to look at the walls for Navigational Targets

            //Step 4.2: OPTIONAL: See if you can find a Navigation Target in front of you.

                // UNCOMMENT AS WE PROGRESS
                /** vuforiaNR.findLocation();
                VectorF translation1 = vuforiaNR.getTranslation();
                Orientation rotation1 = vuforiaNR.getRotation();

                telemetry.addData("Pos (in)", "{X, Y, Z} = %.1f, %.1f, %.1f",
                        translation1.get(0) / mmPerInch, translation1.get(1) / mmPerInch, translation1.get(2) / mmPerInch);
                telemetry.addData("Navigation Found" , vuforiaNR.getTrackableName());
                telemetry.update();
                **/

             sleep(6000);
        }

        vuforiaNR.shutdown();
        telemetry.addData("Status", "Run Time: " + runtime.seconds());
        telemetry.update();
//        baseRobot.StopRobot(null);
    }

}
