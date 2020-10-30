package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="Tank_Drive", group="TeleOp")
public class Tank_Drive extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private NormalizedColorSensor colorSensor=null;
    private double maxPower = 0.3;
    private DcMotorSimple leftIntake = null;
    private DcMotorSimple rightIntake = null;
    private double leftIntakePower = 0;
    private double rightIntakePower = 0;
    private boolean intake = false;
    private DcMotorSimple RotateIntake = null;
    private double leftRotatePower = 0;
    private double rightRotatePower = 0;
    private boolean rotate = false;
    private ColorSensor colorSensor_1;
    private ColorSensor colorSensor_2;
    private DistanceSensor sensorRange;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // values is a reference to the hsvValues array.
        float[] hsvValues = new float[3];
        final float values[] = hsvValues;


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

        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;

        colorSensor_1 = hardwareMap.get(ColorSensor.class,"coloursensor_1");
        colorSensor_1.enableLed(true);
        colorSensor_2 = hardwareMap.get(ColorSensor.class,"coloursensor_2");
        colorSensor_2.enableLed(true);

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.FORWARD);
        RotateIntake.setDirection(DcMotorSimple.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double frontleftPower;
            double frontrightPower;
            double drive_Left = -gamepad1.left_stick_y;
            double drive_Right = -gamepad1.right_stick_y;

            if(gamepad1.left_bumper) {
                maxPower = 1.0;
            } else {
                maxPower = 1.0;
            }

            frontleftPower   = Range.clip(drive_Left,-1.0*maxPower, maxPower) ;
            frontrightPower   = Range.clip(drive_Right, -1.0*maxPower, maxPower) ;

            // Send calculated power to wheels
            leftfrontDrive.setPower(frontleftPower);
            rightfrontDrive.setPower(frontrightPower);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", frontleftPower, frontrightPower);
            telemetry.update();
        }
    }
}
