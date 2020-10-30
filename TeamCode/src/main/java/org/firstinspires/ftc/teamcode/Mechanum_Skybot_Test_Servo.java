package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;


/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpMode
 * s appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="Mechanum_Skybot_Test_Servo", group="TeleOp")
public class Mechanum_Skybot_Test_Servo extends LinearOpMode {
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
    private Servo servoArm = null;
    private double servoArmDown =1;
    private double servoArmUp =0;

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
        servoArm = hardwareMap.get(Servo.class, "servoarm");
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
        servoArm.setPosition(servoArmDown);
        servoArm.setDirection(Servo.Direction.REVERSE) ;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double backleftPower;
            double backrightPower;
            double frontleftPower;
            double frontrightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            // - This uses basic math to combine motions and is easier to drive straight.
            // double drive = gamepad1.left_stick_y;
            // double turn  = -gamepad1.right_stick_x;
            // leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            // rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            // leftPower  = -gamepad1.left_stick_y ;
            // rightPower = -gamepad1.right_stick_y ;
            // Mechanum drive to go side ways
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            double mechanum = gamepad1.right_stick_x;

            if(gamepad1.left_bumper) {
                maxPower = 0.6;
            } else {
                maxPower = 0.3;
            }

            backleftPower   = Range.clip(drive + turn - mechanum, -1.0*maxPower, maxPower) ;
            backrightPower   = Range.clip(drive - turn + mechanum, -1.0*maxPower, maxPower) ;
            frontleftPower   = Range.clip(drive + turn + mechanum, -1.0*maxPower, maxPower) ;
            frontrightPower   = Range.clip(drive - turn - mechanum, -1.0*maxPower, maxPower) ;

            // Send calculated power to wheels
            leftbackDrive.setPower(backleftPower);
            rightbackDrive.setPower(backrightPower);
            leftfrontDrive.setPower(frontleftPower);
            rightfrontDrive.setPower(frontrightPower);

            //Servo for Capestone preload
            if(gamepad1.a){
                servoArm .setPosition(servoArmDown);
            }
            else{
                servoArm.setPosition(servoArm.getPosition());
            }
            if(gamepad1.b){
                servoArm .setPosition(servoArmUp);
            }

            //set up intake power
            if(gamepad2.y) {
                intake = true;
                leftIntakePower = 0.6;
                rightIntakePower = 0.6;
                leftIntake.setPower(leftIntakePower);
                rightIntake.setPower(rightIntakePower);
            } else {
                leftIntakePower = 0;
                rightIntakePower = 0;
            }
            if(gamepad2.x){
                intake = true;
                leftIntakePower = -1.0;
                rightIntakePower = -1.0;
                leftIntake.setPower(leftIntakePower);
                rightIntake.setPower(rightIntakePower);
            } else {
                leftIntakePower = 0;
                rightIntakePower = 0;
            }
            if (gamepad2.back){
                intake = false;
                leftIntakePower = 0;
                rightIntakePower = 0;
                leftIntake.setPower(leftIntakePower);
                rightIntake.setPower(rightIntakePower);
            }
            leftIntake.setPower(0);
            rightIntake.setPower(0);

            //setup rotate power and buttons

            if(gamepad2.a) {
                rotate = true;
                rightRotatePower = 0.8;
                RotateIntake.setPower(rightRotatePower);
            } else {
                leftRotatePower = 0;
                rightRotatePower = 0;
            }
            if(gamepad2.b){
                rotate = true;
                rightRotatePower = -0.6;
                RotateIntake.setPower(rightRotatePower);
            } else {
                leftRotatePower = 0;
                rightRotatePower = 0;
            }
            if (gamepad2.start){
                rotate = false;
                rightRotatePower = 0;
                RotateIntake.setPower(rightRotatePower);
            }
            RotateIntake.setPower(0);

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", frontleftPower, frontrightPower);
            telemetry.addData("BackMotors", "left (%.2f), right (%.2f)", backleftPower, backrightPower);

            telemetry.update();
        }
    }
}

