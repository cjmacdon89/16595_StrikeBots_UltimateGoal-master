package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name="Auto_Grab_Block_Blue", group="")
public class Auto_Grab_Block_Blue extends LinearOpMode {

    // Variable declaration
    private DistanceSensor sensorRange;
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotorSimple rightRotate = null;
    private DcMotorSimple leftRotate = null;
    private DcMotorSimple RotateIntake = null;
    private double RotatePower = 0;
    private double Distance_Measure = 1000;
    private double Colour_Measure_1 = 0;
    private double Colour_Measure_2 = 0;
    private double Colour_Red = 0;
    private double Colour_Blue = 0;
    private double Colour_Green = 0;
    private double Brain = 3;
    private double timelenght = (3000/4*3);
    private ElapsedTime timmer = new ElapsedTime();
    private ColorSensor colorSensor_1;
    private ColorSensor colorSensor_2;
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double globalAngle, power = .30, correction;
    boolean aButton, bButton;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor) sensorRange;

        colorSensor_1 = hardwareMap.get(ColorSensor.class, "coloursensor_1");
        colorSensor_1.enableLed(true);
        colorSensor_2 = hardwareMap.get(ColorSensor.class, "coloursensor_2");
        colorSensor_2.enableLed(true);
        // Set Motors for drive and intake rotation
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        rightRotate = hardwareMap.get(DcMotorSimple.class, "leftIntake");
        leftRotate = hardwareMap.get(DcMotorSimple.class, "rightIntake");
        RotateIntake = hardwareMap.get(DcMotorSimple.class, "RotateIntake");

        // Set motor direction so moving proper direction
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        RotateIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        rightRotate.setDirection(DcMotorSimple.Direction.FORWARD);
        leftRotate.setDirection(DcMotorSimple.Direction.REVERSE);

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

        // Drive Forward Unitl within X distance
        timmer.reset();
        while (Distance_Measure > 10) {
            correction = checkDirection();
            leftbackDrive.setPower(0.2 - correction);
            rightbackDrive.setPower(0.2 + correction);
            leftfrontDrive.setPower(0.2 - correction);
            rightfrontDrive.setPower(0.2 + correction);
            Distance_Measure = sensorRange.getDistance(DistanceUnit.CM);
        }

        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        sleep(750);

        // Check to see if skystone is left, right or center
        Colour_Blue = colorSensor_1.blue();
        Colour_Green = colorSensor_1.green();
        Colour_Red = colorSensor_1.red();
        Colour_Measure_1 = (Colour_Red) / (Colour_Blue + Colour_Red + Colour_Green);

        if (Colour_Measure_1 < 0.32) {

            Brain = 1;
            timmer.reset();
            while (timmer.milliseconds() < 500) {
                correction = checkDirection();
                leftbackDrive.setPower(0.3 - correction);
                rightbackDrive.setPower(-0.3 + correction);
                leftfrontDrive.setPower(-0.3 - correction);
                rightfrontDrive.setPower(0.3 + correction);
            }

        }

        // Check to see if skystone is left, right or center
        Colour_Blue = colorSensor_2.blue();
        Colour_Green = colorSensor_2.green();
        Colour_Red = colorSensor_2.red();
        Colour_Measure_2 = (Colour_Red) / (Colour_Blue + Colour_Red + Colour_Green);

        if (Colour_Measure_2 < 0.35) {
            Brain = 2;
            timmer.reset();
            while (timmer.milliseconds() < 600) {
                correction = checkDirection();
                leftbackDrive.setPower(-0.3 - correction);
                rightbackDrive.setPower(0.3 + correction);
                leftfrontDrive.setPower(0.3 - correction);
                rightfrontDrive.setPower(-0.3 + correction);
            }

        }

        timmer.reset();
        while (timmer.milliseconds() < 800) {
            correction = checkDirection();
            leftbackDrive.setPower(-0.15 - correction);
            rightbackDrive.setPower(-0.15 + correction);
            leftfrontDrive.setPower(-0.15 - correction);
            rightfrontDrive.setPower(-0.15 + correction);
        }

        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);

        //Move arm down
        RotatePower = -0.3;
        RotateIntake.setPower(RotatePower);
        sleep(750);
        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        leftRotate.setPower(0.75);
        rightRotate.setPower(0.75);

        // Move Forward to blocks
        timmer.reset();
        while (timmer.milliseconds() < 3000) {
            correction = checkDirection();
            leftbackDrive.setPower(0.15 - correction);
            rightbackDrive.setPower(0.15 + correction);
            leftfrontDrive.setPower(0.15 - correction);
            rightfrontDrive.setPower(0.15 + correction);
        }

        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        leftRotate.setPower(0);
        rightRotate.setPower(0);

        timmer.reset();
        while (timmer.milliseconds() < 1100) {
            correction = checkDirection();
            leftbackDrive.setPower(-0.3 - correction);
            rightbackDrive.setPower(-0.3 + correction);
            leftfrontDrive.setPower(-0.3 - correction);
            rightfrontDrive.setPower(-0.3 + correction);
        }

        if (Brain == 2) {

            timelenght = (3400/4*3);

        }
        if (Brain == 1) {

            timelenght = (2500/4*3);

        }
        //goes under brige.
        timmer.reset();
        while (timmer.milliseconds() < timelenght) {
            correction = checkDirection();
            leftbackDrive.setPower(0.45- correction);
            rightbackDrive.setPower(-0.45 + correction);
            leftfrontDrive.setPower(-0.45 - correction);
            rightfrontDrive.setPower(0.45 + correction);
        }
        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        leftRotate.setPower(-0.75);
        rightRotate.setPower(-0.75);
        sleep(500);
        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        leftRotate.setPower(-0.0);
        rightRotate.setPower(-0.0);

        timmer.reset();
        while (timmer.milliseconds() < timelenght+1600){
            correction = checkDirection();
            leftbackDrive.setPower(-0.45 - correction);
            rightbackDrive.setPower(0.45 + correction);
            leftfrontDrive.setPower(0.45 - correction);
            rightfrontDrive.setPower(-0.45 + correction);
        }
        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        leftRotate.setPower(0.75);
        rightRotate.setPower(0.75);

        if (Brain == 2) {
            timmer.reset();
            while (timmer.milliseconds() < 1800) {
                correction = checkDirection();
                leftbackDrive.setPower(0.15- correction);
                rightbackDrive.setPower(0.15 + correction);
                leftfrontDrive.setPower(0.15 - correction);
                rightfrontDrive.setPower(0.15 + correction);
            }
            timmer.reset();
            while (timmer.milliseconds() < 1200) {
                correction = checkDirection();
                leftbackDrive.setPower(0.2);
                rightbackDrive.setPower(0.1);
                leftfrontDrive.setPower(0.2);
                rightfrontDrive.setPower(0.1);
            }
            timmer.reset();
            while (timmer.milliseconds() < 200) {
                correction = checkDirection();
                leftbackDrive.setPower(-0.15);
                rightbackDrive.setPower(-0.14);
                leftfrontDrive.setPower(-0.15);
                rightfrontDrive.setPower(-0.14);
            }
        }
        if (Brain < 2 || Brain > 2) {
            // Move Forward to blocks
            timmer.reset();
            while (timmer.milliseconds() < 2700) {
                correction = checkDirection();
                leftbackDrive.setPower(0.15 - correction);
                rightbackDrive.setPower(0.15 + correction);
                leftfrontDrive.setPower(0.15 - correction);
                rightfrontDrive.setPower(0.15 + correction);

            }
        }
        timmer.reset();
        while (timmer.milliseconds() < 1000) {
            correction = checkDirection();
            leftbackDrive.setPower(-0.3 - correction);
            rightbackDrive.setPower(-0.3 + correction);
            leftfrontDrive.setPower(-0.3 - correction);
            rightfrontDrive.setPower(-0.3 + correction);
        }
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        leftRotate.setPower(0);
        rightRotate.setPower(0);

        timmer.reset();
        while (timmer.milliseconds() < timelenght+1000) {
            correction = checkDirection();
            leftbackDrive.setPower(0.45 - correction);
            rightbackDrive.setPower(-0.45 + correction);
            leftfrontDrive.setPower(-0.45 - correction);
            rightfrontDrive.setPower(0.45 + correction);
        }

        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        leftRotate.setPower(-0.75);
        rightRotate.setPower(-0.75);

        timmer.reset();
        while (timmer.milliseconds() < (1000)) {
            correction = checkDirection();
            leftbackDrive.setPower(-0.45 - correction);
            rightbackDrive.setPower(0.45 + correction);
            leftfrontDrive.setPower(0.45 - correction);
            rightfrontDrive.setPower(-0.45 + correction);
        }


        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        leftRotate.setPower(0);
        rightRotate.setPower(0);

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