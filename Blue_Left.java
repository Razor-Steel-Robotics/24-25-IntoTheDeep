/*
    This is new and needs to be modified to be the mirror of the Red Right code
    Then We need to calibrate the color sensor to detect blue //Possibly corrected


 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Blue Left", group = "Mr. Phil")
public class Blue_Left extends LinearOpMode {
    /**
     * The colorSensor field will contain a reference to our color sensor hardware object
     */


    /**
     * The relativeLayout field is used to aid in providing interesting visual feedback
     * in this sample application; you probably *don't* need this when you use a color sensor on your
     * robot. Note that you won't see anything change on the Driver Station, only on the Robot Controller.
     */
    View relativeLayout;
    // get an instance of the "Robot" class.
    private final SimplifiedOdometryRobot robot = new SimplifiedOdometryRobot(this);
    private NormalizedColorSensor sensorleft;
    private NormalizedColorSensor sensorright;
    private Servo Pix_Pusher = null;
    private Servo box = null;
    private DcMotor lift = null;

    final float[] hsvValues = new float[3];
    @Override
    public void runOpMode() {
        sensorleft = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        Pix_Pusher =hardwareMap.get(Servo.class, "Pix");
        sensorright = hardwareMap.get(NormalizedColorSensor.class, "sensor_colorR");
        lift = hardwareMap.get(DcMotor.class, "lift/lateral");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        box = hardwareMap.get(Servo.class, "box");
        // Wait for driver to press start

        telemetry.speak("This is Blue left Autonomous, ready to start ", "en","US");
        telemetry.addData(">", "Touch Play to run Blue left Autonomous");
        telemetry.update();
        robot.initialize(true);
        Pix_Pusher.setPosition(1);
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto
        double step = 0;
        double colorL = 2;
        double Lblue =0;
        double Rblue = 0;
        double liftpower =0;
        double Rdistence = 0;
        double Ldistaance = 0;
        // Run Auto if stop was not pressed.
        if (opModeIsActive()) {


            //put drive commands here


            if (step < 1) {
                //step 1
                robot.drive(32, .2, .5);
                step = 1;
            }

            if (colorL>1) {
                for (int count = 0; count < 4; count++) {
                    sleep(50);
                    NormalizedRGBA colors = sensorright.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    Lblue = colors.blue; //was colors.red, testing colors.blue
                    if (sensorright instanceof DistanceSensor) {
                        Rdistence = ((DistanceSensor) sensorright).getDistance(DistanceUnit.CM);
                    }
                }
                colorL=0;

            }

            if (colorL<1) {
                for (int count = 0; count < 4; count++) {
                    sleep(50);
                    NormalizedRGBA colors = sensorleft.getNormalizedColors();
                    Color.colorToHSV(colors.toColor(), hsvValues);
                    Rblue = colors.blue; //was colors.red, testing colors.blue
                    if (sensorleft instanceof DistanceSensor) {
                        Ldistaance = ((DistanceSensor) sensorleft).getDistance(DistanceUnit.CM);
                    }
                }
            }






            if (step < 2) {
                if (Rdistence < 5) {//Originally Lred, test Rred
                    telemetry.speak("Pixel Detected, going left","en","US");
                    telemetry.addData(">","going left");
                    telemetry.update();
                    robot.turnTo(90,.3,.5);
                    robot.drive(4,.1,0);
                    Pix_Pusher.setPosition(0);
                    robot.drive(-7,.1,.5);
                    robot.strafe(18, .5, .5);
                    robot.turnTo(90, .3, .5);//was 270, testing 90 right worked
                    robot.drive(30, .5, .5);
                    robot.strafe(-20,.5,.5);//change required
                    robot.drive(8,.2,0);//change required
                    lift.setPower(-.5);
                    sleep(700);
                    lift.setPower(0);
                    box.setPosition(1);
                    sleep(1500);
                    box.setPosition(.5);
                    robot.drive(5,.2,.5);
                    robot.strafe(15,.3,.2);//change required /was -30/
                    robot.drive(13,.3,.2);
                    sleep(3000);

                } else if (Ldistaance <5) {//Originally Rred, testing Lred
                    telemetry.speak("Pixel Detected, going right","en","US");
                    telemetry.addData(">","going right");
                    telemetry.update();
                    robot.turnTo(270,.3,.5);
                    robot.drive(4,.1,.5);
                    Pix_Pusher.setPosition(0);
                    robot.drive(-7,.1,.5);
                    robot.strafe(18, .4, .5);
                    robot.turnTo(90, .3, .5);//change 270 to 90
                    robot.drive(30, .5, .5);
                    robot.strafe(-15,.5,.5);//change
                    robot.drive(4,.2,0);//change
                    lift.setPower(-.5);
                    sleep(700);
                    lift.setPower(0);
                    box.setPosition(1);
                    sleep(2000);
                    box.setPosition(.5);
                    robot.drive(-5,.2,.5);
                    robot.strafe(-15,.3,.2);//change -35 to 15
                    robot.drive(13,.3,.2);
                    sleep(30000);

                }else {//Assumed to be working based on Red Left configuration
                    telemetry.speak("Pixel not Detected, going middle","en","US");
                    telemetry.addData(">","going middle");
                    telemetry.update();
                    Pix_Pusher.setPosition(0);
                    robot.drive(-14,.3,.5);
                    robot.turnTo(90, .3, .5);//change 270 to 90
                    robot.drive(35, .5, .5);
                    robot.strafe(5,.5,.5);//change 5 to -5
                    robot.drive(4,.2,0);
                    lift.setPower(-.5);
                    sleep(700);
                    lift.setPower(0);
                    box.setPosition(1);
                    sleep(1500);
                    box.setPosition(.5);
                    robot.drive(-5,.2,.5);
                    robot.strafe(-30,.3,.2);//change -30 to 30
                    robot.drive(13,.3,.2);

                    sleep(30000);
                }

            }


        }
    }
}