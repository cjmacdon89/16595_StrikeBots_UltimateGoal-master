package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="AutoUltimateGoal", group="")
public class AutoUltimateGoal extends LinearOpMode {

    // Variable declaration
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


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
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

        // wait for start button.
        waitForStart();

        telemetry.addData("Mode", "running");
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

        // Drive Forward for X seconds to drop wobble
        timmer.reset();
        while (timmer.milliseconds() < 1000) {
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
        // Mechanium to drop wobble goal
        timmer.reset();
        while (timmer.milliseconds() < 2000) {
            correction = checkDirection();
            leftbackDrive.setPower(power_level - correction);
            rightbackDrive.setPower(-power_level + correction);
            leftfrontDrive.setPower(-power_level - correction);
            rightfrontDrive.setPower(power_level+ correction);
        }
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        servoGrab.setPosition(servoGrabOpen);
        sleep(500);

        // Drive back to line to park
        timmer.reset();
        while (timmer.milliseconds() < 400) {
            correction = checkDirection();
            leftbackDrive.setPower(-power_level - correction);
            rightbackDrive.setPower(-power_level + correction);
            leftfrontDrive.setPower(-power_level - correction);
            rightfrontDrive.setPower(-power_level + correction);
        }

        // turn off motor power
        leftbackDrive.setPower(power_zero);
        rightbackDrive.setPower(power_zero);
        leftfrontDrive.setPower(power_zero);
        rightfrontDrive.setPower(power_zero);
        leftRotate.setPower(power_zero);
        rightRotate.setPower(power_zero);

    }

    // Function to get angle change from gyro
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
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
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

        return correction;
    }
}