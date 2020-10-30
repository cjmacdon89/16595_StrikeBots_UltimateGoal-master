package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name="ColorTest", group="TeleOp")
public class ColorSensorTest extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private double maxPower = 0.25;
    private Servo servoArm = null;
    private double servoArmDown =0.5;
    private double servoArmUp =-0.5;
    private DcMotorSimple leftIntake = null;
    private DcMotorSimple rightIntake = null;
    private double leftIntakePower = 0;
    private double rightIntakePower = 0;
    private boolean intake = false;
    private double Colour_Measure_1 = 0;
    private double Colour_Measure_2 = 0;
    private double Colour_Measure_3 = 0;
    private double Colour_Red = 0;
    private double Colour_Blue = 0;
    private double Colour_Green = 0;
    private ColorSensor colorSensor_1;
    private ColorSensor colorSensor_2;
    private DistanceSensor sensorRange;
    private double Distance_Measure = 1000;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        colorSensor_1 = hardwareMap.get(ColorSensor.class,"coloursensor_1");
        colorSensor_1.enableLed(true);
        colorSensor_2 = hardwareMap.get(ColorSensor.class,"coloursensor_2");
        colorSensor_2.enableLed(true);
        // Read the sensor
        sensorRange = hardwareMap.get(DistanceSensor.class, "sensor_range");
        Rev2mDistanceSensor sensorTimeOfFlight = (Rev2mDistanceSensor)sensorRange;
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        colorSensor_2.enableLed(true);
        // Check to see if skystone is left, right or center
        Colour_Blue= colorSensor_1.blue();
        Colour_Green = colorSensor_1.green();
        Colour_Red = colorSensor_1.red();
        Colour_Measure_1 = (Colour_Red)/(Colour_Blue+Colour_Red+Colour_Green);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            Colour_Blue= colorSensor_2.blue();
            Colour_Green = colorSensor_2.green();
            Colour_Red = colorSensor_2.red();
            Colour_Measure_1 = (Colour_Red)/(Colour_Blue+Colour_Red+Colour_Green);
            Colour_Measure_2 = (Colour_Green)/(Colour_Blue+Colour_Red+Colour_Green);
            Colour_Measure_3 = (Colour_Green)/(Colour_Blue+Colour_Red+Colour_Green);
            Distance_Measure = sensorRange.getDistance(DistanceUnit.CM);
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addLine("raw Android color: ")
                    .addData("a", "%02d", colorSensor_2.alpha())
                    .addData("r", "%02d", colorSensor_2.red())
                    .addData("g", "%02d", colorSensor_2.green())
                    .addData("b", "%02d", colorSensor_2.blue());
            telemetry.addData("Colour_Norm_Red","%.2f",Colour_Measure_1);
            telemetry.addData("Colour_Norm_Green","%.2f",Colour_Measure_2);
            telemetry.addData("Colour_Norm_Blue","%.2f",Colour_Measure_3);
            telemetry.addData("Distance","%.2f",Distance_Measure);
            telemetry.update();
        }
    }
}

