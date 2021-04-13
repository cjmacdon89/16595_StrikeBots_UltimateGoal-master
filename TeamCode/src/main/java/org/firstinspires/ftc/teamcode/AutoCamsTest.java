/* Copyright (c) 2019 FIRST. All rights reserved.
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

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import java.util.List;



import java.util.List;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Ultimate Goal game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "AutoCamsTest", group = "Concept")

public class AutoCamsTest extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "UltimateGoal.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Quad";
    private static final String LABEL_SECOND_ELEMENT = "Single";
    private double ringcondition = 5;
    private double WereShouldIDropWobble = 0;
    private ElapsedTime timmer = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotorSimple leftIntake = null;
    private DcMotorSimple rightIntake = null;
    private DcMotorSimple RotateIntake = null;
    private Servo servoGrab = null;
    private double servoGrabOpen =0.8;
    private double servoGrabClose =0.135;
    private Servo servoHold = null;
    private double servoHoldOpen =1;
    private double servoHoldClose =0;
    private DcMotorSimple rightRotate = null;
    private DcMotorSimple leftRotate = null;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, correction;
    private double power_level = 0.5;
    private double power_zero = 0;
    private double shooter_speed = -0.535;
    private double intake_speed = 1.0;

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY = "AYBs0TP/////AAABmeSKs9CeHUZJgmAYSDe8df8bqYt5JpDHAqKhRw6zqvIwqKdDYwdDQCPhbFjj/onpkKsiY8VIL30/T5XuXBEXIxGT3L+2wjvyfx2B0/0HJKlRdjqUXfkYEi3eVxZXBqLFapMjXLyORUBsGHmC7v0MATBm3CYXX/7UhhrMU9tMUPbTQd1cZiq5HbrJXr8c+ZfsrneZ7B7hdMmZsOGT4mWkm9aL/E/2aSYiHm+5/3B9T8NUiW0YuzvE6sfD6KVaj0fRAwrhDmxNJzZMEazQrE7D3my0yYyR80o0Lm143BDtYjdMs1eaeIT5GGSOjxDJfn4yXq1Fb3DJXryQzv0SA5fHAKnEX71CAFdD4Ym0KAKiHRDH";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.


        //Set up motors
        leftbackDrive  = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");
        leftfrontDrive  = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        leftIntake = hardwareMap.get(DcMotorSimple.class, "leftIntake");
        rightIntake = hardwareMap.get(DcMotorSimple.class, "rightIntake");
        RotateIntake = hardwareMap.get(DcMotorSimple.class, "RotateIntake");
        servoGrab = hardwareMap.get(Servo.class, "servoGrab");
        servoHold = hardwareMap.get(Servo.class, "servoHold");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery

        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        RotateIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        servoGrab.setPosition(servoGrabOpen);
        servoGrab.setDirection(Servo.Direction.REVERSE) ;
        servoHold.setPosition(servoHoldClose);
        servoHold.setDirection(Servo.Direction.REVERSE) ;

        // Setup Gyro
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        // Retrieve and initialize the IMU. We expect the IMU to be attached to an I2C port
        // on a Core Device Interface Module, configured to be a sensor of type "AdaFruit IMU",
        // and named "imu".
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);
        telemetry.addData("Mode", "calibrating...");
        telemetry.update();

        // make sure the imu gyro is calibrated before continuing.
        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        telemetry.addData("Mode", "waiting for start");
        telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        telemetry.update();

        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            // The TensorFlow software will scale the input images from the camera to a lower resolution.
            // This can result in lower detection accuracy at longer distances (> 55cm or 22").
            // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
            // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
            // should be set to the value of the images used to create the TensorFlow Object Detection model
            // (typically 1.78 or 16/9).

            // Uncomment the following line if you want to adjust the magnification and/or the aspect ratio of the input images.
            //tfod.setZoom(2.5, 1.78);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        timmer.reset();

        while (opModeIsActive()) {


            while (timmer.milliseconds() < 5000) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        if (updatedRecognitions.size() < 1) {
                            ringcondition = 0;
                            WereShouldIDropWobble = 200;
                        }
                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());

                            if (recognition.getLabel() == "Single") {
                                ringcondition = 1;
                                WereShouldIDropWobble = 500;
                            }
                            if (recognition.getLabel() == "Quad") {
                                ringcondition = 4;
                                WereShouldIDropWobble = 800;
                            }
                        }
                        telemetry.update();
                    }

                }

            }
            if (tfod != null) {
                tfod.shutdown();
            }

            telemetry.addData("State", "Ring Seen (%.2f)", ringcondition);
            telemetry.update();

            // Drive Forward to shoot
            timmer.reset();
            while (timmer.milliseconds() < 1250) {
                correction = checkDirection();
                leftbackDrive.setPower(power_level - correction);
                rightbackDrive.setPower(power_level + correction);
                leftfrontDrive.setPower(power_level - correction);
                rightfrontDrive.setPower(power_level + correction);
                rightIntake.setPower(shooter_speed);
            }

            timmer.reset();
            while (timmer.milliseconds() < 650) {
                correction = checkDirection();
                leftbackDrive.setPower(-power_level - correction);
                rightbackDrive.setPower(power_level + correction);
                leftfrontDrive.setPower(power_level - correction);
                rightfrontDrive.setPower(-power_level + correction);
            }

            //First shot
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            sleep(500);
            leftIntake.setPower(intake_speed);
            sleep(400);
            leftIntake.setPower(0);

            timmer.reset();
            while (timmer.milliseconds() < 200) {
                correction = checkDirection();
                leftbackDrive.setPower(-power_level - correction);
                rightbackDrive.setPower(power_level + correction);
                leftfrontDrive.setPower(power_level - correction);
                rightfrontDrive.setPower(-power_level + correction);
            }
            //Second shot
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            sleep(200);
            leftIntake.setPower(intake_speed);
            sleep(500);
            leftIntake.setPower(0);

            timmer.reset();
            while (timmer.milliseconds() < 200) {
                correction = checkDirection();
                leftbackDrive.setPower(-power_level - correction);
                rightbackDrive.setPower(power_level + correction);
                leftfrontDrive.setPower(power_level - correction);
                rightfrontDrive.setPower(-power_level + correction);
            }
            //Third shot
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            leftIntake.setPower(intake_speed);
            sleep(1500);
            leftIntake.setPower(0);
            rightIntake.setPower(0);

            //Getting on 1 of axis of the square were the wobbles should go
            timmer.reset();
            while (timmer.milliseconds() < WereShouldIDropWobble) {
                correction = checkDirection();
                leftbackDrive.setPower(power_level - correction);
                rightbackDrive.setPower(power_level + correction);
                leftfrontDrive.setPower(power_level - correction);
                rightfrontDrive.setPower(power_level + correction);
            }

            // Stop and drop wobble goal
            leftbackDrive.setPower(0);
            rightbackDrive.setPower(0);
            leftfrontDrive.setPower(0);
            rightfrontDrive.setPower(0);
            sleep(750);
            servoHold.setPosition(servoHoldClose);
            // Mechanum to drop wobble goal
            timmer.reset();
            while (timmer.milliseconds() < 2000) {
                correction = checkDirection();
                leftbackDrive.setPower(power_level - correction);
                rightbackDrive.setPower(-power_level + correction);
                leftfrontDrive.setPower(-power_level - correction);
                rightfrontDrive.setPower(power_level+ correction);
            }


            // Drive back to line to park
            timmer.reset();
            while (timmer.milliseconds() < WereShouldIDropWobble-190) {
                correction = checkDirection();
                leftbackDrive.setPower(-power_level - correction);
                rightbackDrive.setPower(-power_level + correction);
                leftfrontDrive.setPower(-power_level - correction);
                rightfrontDrive.setPower(-power_level + correction);
            }
        }

    }



    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    private double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }
    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minResultConfidence = 0.8f;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

    private double checkDirection() {
            // The gain value determines how sensitive the correction is to direction changes.
            // You will have to experiment with your robot to get small smooth direction changes
            // to stay on a straight line.
            double correction, angle, gain = .02;

            angle = getAngle();

            if (angle == 0)
                correction = 0;             // no adjustment.
            else
                correction = -angle;        // reverse sign of angle for correction.

            correction = correction * gain;
            correction = 0;
            return correction;

        }
    }
