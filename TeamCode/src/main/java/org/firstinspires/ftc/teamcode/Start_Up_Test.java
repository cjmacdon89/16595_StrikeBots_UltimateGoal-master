package org.firstinspires.ftc.teamcode;

// Autonomous program to grab tray and but into building zone then park robot under bridge.
//Program uses timmer for distance control and gyro to correct path

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Autonomous(name="Start_Up_Test", group="")
public class Start_Up_Test extends LinearOpMode {

    // Variable declaration
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private DcMotorSimple rightRotate = null;
    private DcMotorSimple leftRotate = null;
    private DcMotorSimple RotateIntake = null;
    private double RotatePower = 0;
    private ElapsedTime timmer = new ElapsedTime();
    BNO055IMU imu;
    Orientation lastAngles = new Orientation();
    double                  globalAngle, power = .30, correction;
    boolean                 aButton, bButton;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Set Motors for drive and intake rotation
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");
        leftfrontDrive = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");
        RotateIntake = hardwareMap.get(DcMotorSimple.class, "RotateIntake");
        rightRotate = hardwareMap.get(DcMotorSimple.class, "leftIntake");
        leftRotate = hardwareMap.get(DcMotorSimple.class, "rightIntake");

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

        //Turn left
        timmer.reset();
        while (timmer.milliseconds() < (500))
        {
            correction = checkDirection();
            leftbackDrive.setPower(+0.2);
            rightbackDrive.setPower(-0.2 );
            leftfrontDrive.setPower(+0.2 );
            rightfrontDrive.setPower(-0.2);
        }
        //Turn right
        timmer.reset();
        while (timmer.milliseconds() < (500))
        {
            correction = checkDirection();
            leftbackDrive.setPower(-0.2);
            rightbackDrive.setPower(+0.2 );
            leftfrontDrive.setPower(-0.2 );
            rightfrontDrive.setPower(+0.2);
        }

        //place down arm
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        RotatePower = -0.3;
        RotateIntake.setPower(RotatePower);
        sleep(750);
        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        sleep(1000);

        leftRotate.setPower(1.0);
        rightRotate.setPower(1.0);
        sleep(750);
        leftRotate.setPower(0);
        rightRotate.setPower(0);

        //lift arm up
        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
        RotatePower = 0.6;
        RotateIntake.setPower(RotatePower);
        sleep(500);
        RotatePower = 0;
        RotateIntake.setPower(RotatePower);
        sleep(1000);


        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);

    }

    // Function to get angle change from gyro
    private double getAngle()
    {
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
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection()
    {
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