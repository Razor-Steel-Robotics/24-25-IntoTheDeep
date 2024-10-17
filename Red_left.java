/* Created by Phil Malone. 2023.
    This class illustrates my simplified Odometry Strategy.
    It implements basic straight line motions but with heading and drift controls to limit drift.
    See the readme for a link to a video tutorial explaining the operation and limitations of the code.
 */

package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import android.view.View;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/*
 * This OpMode illustrates an autonomous opmode using simple Odometry
 * All robot functions are performed by an external "Robot" class that manages all hardware interactions.
 * Pure Drive or Strafe motions are maintained using two Odometry Wheels.
 * The IMU gyro is used to stabilize the heading during all motions
 */

@Autonomous(name="Red left", group = "Mr. Phil")
public class Red_left extends LinearOpMode {
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
    private Servo airplane = null;

    final float[] hsvValues = new float[3];
    @Override
    public void runOpMode() {
        sensorleft = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");
        airplane = hardwareMap.get(Servo.class, "Airplane");
        Pix_Pusher =hardwareMap.get(Servo.class, "Pix");
        sensorright = hardwareMap.get(NormalizedColorSensor.class, "sensor_colorR");
        lift = hardwareMap.get(DcMotor.class, "lift/lateral");
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        box = hardwareMap.get(Servo.class, "box");
        // Wait for driver to press start
        telemetry.speak("This is Red Left Autonomous, ready to start ", "en","US");
        telemetry.addData(">", "Touch Play to run red left Autonomous");
        telemetry.update();
        robot.initialize(true);
        Pix_Pusher.setPosition(1);
        airplane.setPosition(1);
        waitForStart();
        robot.resetHeading();  // Reset heading to set a baseline for Auto
        double step = 0;
        double colorL = 2;
        double Lred =0;
        double Rred = 0;
        double liftpower =0;
        double Ldistaance = 0;
        double Rdistance  =0;
        // Run Auto if stop was not pressed.
        if (opModeIsActive()) {
            //put drive commands here

                if (step < 1) {
                    //step 1
                    robot.drive(-32, .2, .5);
                    step = 1;
                }

                if (colorL>1) {
                    for (int count = 0; count < 4; count++) {
                        sleep(500);
                        NormalizedRGBA colors = sensorright.getNormalizedColors();
                        Color.colorToHSV(colors.toColor(), hsvValues);
                        //Rred = colors.green;
                        if (sensorleft instanceof DistanceSensor) {
                            Ldistaance = ((DistanceSensor) sensorleft).getDistance(DistanceUnit.CM);
                        }
                    }
                    colorL=0;

                }

                if (colorL<1) {
                    for (int count = 0; count < 4; count++) {
                        sleep(500);
                        NormalizedRGBA colors = sensorleft.getNormalizedColors();
                        Color.colorToHSV(colors.toColor(), hsvValues);
                        Lred = colors.red;
                        Rdistance = ((DistanceSensor) sensorright).getDistance(DistanceUnit.CM);
                    }
                }
                if (step < 2) {
                    if (Rdistance < 6) {//Lred original orientation, Fixed by swapping Rred
                        telemetry.speak("Pixel Detected, going left","en","US");
                        telemetry.addData(">","going left");
                        telemetry.update();
                        robot.turnTo(271,.3,.5); //Turns left
                        robot.drive(4,.1,.5); //Moves Forward
                        Pix_Pusher.setPosition(0);//Releases pixel
                        robot.drive(-5,.1,.5);//Drives backwards
                        robot.strafe(-18, .5, .5);//Strafes sideways
                        robot.turnTo(90, .3, .5);//Turn /was 89.5 testing 90
                        robot.drive(80, .5, .5);
                        robot.strafe(-14,.5,.5);
                        robot.drive(6,.2,0);
                        lift.setPower(-.5);
                        sleep(400);
                        box.setPosition(1);
                        lift.setPower(0);
                        sleep(500);
                        lift.setPower(-1);
                        sleep(150);
                        lift.setPower(0);
                        sleep(2000);
                        box.setPosition(.5);
                        sleep(1500);
                        box.setPosition(.5);
                        robot.drive(-7,.2,.5);
                        robot.strafe(15,.3,.2);
                        robot.drive(13,.3,.2);
                        sleep(3000);

                    } else if (Ldistaance < 6) {//Rred original orientation, Fixed by swapping to Lred
                        telemetry.speak("Pixel Detected, going right","en","US");
                        telemetry.addData(">","going right");
                        telemetry.update();
                        robot.turnTo(90,.3,.5);
                        robot.drive(2,.1,.5);
                        Pix_Pusher.setPosition(0);
                        robot.drive(-5,.1,.5);
                        robot.strafe(18, .4, .5);
                        robot.turnTo(90, .3, .5);
                        robot.drive(80, .5, .5);
                        robot.strafe(-27,.5,.5);
                        robot.drive(12,.2,0);
                        lift.setPower(-.5);
                        sleep(400);
                        box.setPosition(1);
                        lift.setPower(0);
                        sleep(500);
                        lift.setPower(-1);
                        sleep(150);
                        lift.setPower(0);
                        sleep(2000);
                        box.setPosition(.5);
                        robot.drive(-7,.2,.5);
                        robot.strafe(30,.3,.2);
                        robot.drive(13,.3,.2);
                        sleep(30000);

                    }else {//Middle option, goes middle (works already as default)
                        telemetry.speak("Pixel not Detected, going middle","en","US");
                        telemetry.addData(">","going middle");
                        telemetry.update();
                        robot.drive(-14,.3,.5);
                        Pix_Pusher.setPosition(0);
                        robot.drive(-5,.3,.5);
                        robot.turnTo(90, .3, .5);
                        robot.drive(80, .5, .5);
                        robot.strafe(-20,.5,.5);
                        robot.drive(10,.2,0);
                        lift.setPower(-.5);
                        sleep(400);
                        box.setPosition(1);
                        lift.setPower(0);
                        sleep(500);
                        lift.setPower(-1);
                        sleep(150);
                        lift.setPower(0);
                        sleep(2000);
                        box.setPosition(.5);
                        robot.drive(-7,.2,.5);
                        robot.strafe(25,.3,.2);
                        robot.drive(13,.3,.2);

                        sleep(30000);
                    }
                }


        }
    }
}