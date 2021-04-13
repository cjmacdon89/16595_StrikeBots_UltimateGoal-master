package org.firstinspires.ftc.teamcode.Autonomous;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


//@TeleOp(name="Elevator_Test", group="TeleOp")
public class Elevator_Test extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private double elevator_motion;
    private Servo servoArm = null;
    private double servoArmDown =0.5;
    private double servoArmUp =0.15;
    private Servo servoHolder = null;
    private double servoHolderDown =1;
    private double servoHolderUp =0;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftbackDrive = hardwareMap.get(DcMotor.class, "leftbackdrive");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);

        servoArm = hardwareMap.get(Servo.class, "servoarm");
        servoArm.setPosition(servoArmDown);
        servoArm.setDirection(Servo.Direction.REVERSE) ;

        servoHolder = hardwareMap.get(Servo.class, "servoHolder");
        servoHolder.setPosition(servoHolderDown);
        servoHolder.setDirection(Servo.Direction.REVERSE) ;

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            if (gamepad1.a) {
                elevator_motion = 1.0;
            }

            if (gamepad1.y) {
                    elevator_motion = -1.0;
            }

            leftbackDrive.setPower(elevator_motion);

                // Show the elapsed game time and wheel power.
                telemetry.addData("Status", "Run Time: " + runtime.toString());
                telemetry.update();
            elevator_motion = 0;

            if(gamepad1.x){
                servoArm .setPosition(servoArmDown);
            }
            else{
                servoArm.setPosition(servoArm.getPosition());
            }
            if(gamepad1.b){
                servoArm .setPosition(servoArmUp);
            }

            //Just for testing the holder for start of game.

            if(gamepad2.x){
                servoHolder .setPosition(servoHolderDown);
            }
            else{
                servoHolder.setPosition(servoHolder.getPosition());
            }
            if(gamepad2.b){
                servoHolder .setPosition(servoHolderUp);
            }

        }

            // Set power off motors to zero once code ends
            leftbackDrive.setPower(0);
        }

    }


