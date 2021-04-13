package org.firstinspires.ftc.teamcode;

        import com.qualcomm.hardware.bosch.BNO055IMU;
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

        import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
        import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
        import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="Ultimate_Goal_Revised_Driver_Control", group="TeleOp")
public class Ultimate_Goal_Revised_Driver_Control extends LinearOpMode {
    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;
    private double maxPower = 0.65;
    private DcMotorSimple Elavator = null;
    private double ElavatorPower = 0;
    private DcMotorSimple Shooter = null;
    private double ShooterPower = 0;
    private DcMotorSimple TheIntakeSystem = null;
    private double TheIntakeSystemsPower = 0;
    private Servo servoGrab = null;
    private double servoGrabOpen =0.75;
    private double servoGrabClose =0.135;
    private Servo servoHold = null;
    private double servoHoldOpen =1;
    private double servoHoldClose =0;

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
        Elavator = hardwareMap.get(DcMotorSimple.class, "leftIntake");
        Shooter = hardwareMap.get(DcMotorSimple.class, "rightIntake");
        TheIntakeSystem = hardwareMap.get(DcMotorSimple.class, "RotateIntake");
        servoGrab = hardwareMap.get(Servo.class, "servoGrab");
        servoHold = hardwareMap.get(Servo.class, "servoHold");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);
        Elavator.setDirection(DcMotorSimple.Direction.REVERSE);
        Shooter.setDirection(DcMotorSimple.Direction.FORWARD);
        TheIntakeSystem.setDirection(DcMotorSimple.Direction.FORWARD);
        servoGrab.setPosition(servoGrabClose);
        servoGrab.setDirection(Servo.Direction.REVERSE) ;
        servoHold.setPosition(servoHoldClose);
        servoHold.setDirection(Servo.Direction.REVERSE) ;

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

            //Joy sticks given their funstions
            double drive = -gamepad1.left_stick_y;
            double turn  =  gamepad1.left_stick_x;
            double mechanum = gamepad1.right_stick_x;

            //Giving more or less speed for the robot
            if(gamepad1.right_bumper) {
                    maxPower = 0.25;
            } else {
                maxPower = 0.65;
            }

            //Setting wheel power
            backleftPower   = Range.clip(drive + turn - mechanum, -0.75*maxPower, maxPower) ;
            backrightPower   = Range.clip(drive - turn + mechanum, -0.75*maxPower, maxPower) ;
            frontleftPower   = Range.clip(drive + turn + mechanum, -0.75*maxPower, maxPower) ;
            frontrightPower   = Range.clip(drive - turn - mechanum, -0.75*maxPower, maxPower) ;

            // Send calculated power to wheels
            leftbackDrive.setPower(backleftPower);
            rightbackDrive.setPower(backrightPower);
            leftfrontDrive.setPower(frontleftPower);
            rightfrontDrive.setPower(frontrightPower);

            //Intake
            if(gamepad2.y) {
                TheIntakeSystemsPower = 1.0;
                TheIntakeSystem.setPower(TheIntakeSystemsPower);
            }
            if(gamepad2.x){
                TheIntakeSystemsPower = -1.0;
                TheIntakeSystem.setPower(TheIntakeSystemsPower);
            } else {
                TheIntakeSystemsPower = 0;
                TheIntakeSystem.setPower(TheIntakeSystemsPower);
            }

            //Speed of shooter power buttons
            if(gamepad2.left_bumper) {
                ShooterPower = -0.6;
                Shooter.setPower(ShooterPower);
            }
            if(gamepad2.right_bumper){
                ShooterPower = -0.54;
                Shooter.setPower(ShooterPower);
            }
            if(gamepad2.left_trigger > 0.25){
                ShooterPower = -0.58;
                Shooter.setPower(ShooterPower);
            }
            if(gamepad2.right_trigger > 0.25){
                ShooterPower = -0.65;
                Shooter.setPower(ShooterPower);
            } if(gamepad2.start) {
                ShooterPower = 0;
                Shooter.setPower(ShooterPower);
            }

            //Elevator

            if(gamepad2.a) {
                ElavatorPower = 1.0;
                Elavator.setPower(ElavatorPower);
            }
            if(gamepad2.b){
                ElavatorPower = -1.0;
                Elavator.setPower(ElavatorPower);
            } else {
                ElavatorPower = 0;
                Elavator.setPower(ElavatorPower);
            }

            //Servo buttons
            if(gamepad1.a){
                servoGrab.setPosition(servoGrabOpen);
            }
            if(gamepad1.b){
                servoGrab.setPosition(servoGrabClose);
            }
            if(gamepad1.x){
                servoHold.setPosition(servoHoldOpen);
            }
            if(gamepad1.y){
                servoHold.setPosition(servoHoldClose);
            }

            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("FrontMotors", "left (%.2f), right (%.2f)", frontleftPower, frontrightPower);
            telemetry.addData("BackMotors", "left (%.2f), right (%.2f)", backleftPower, backrightPower);
            telemetry.update();
        }
    }
}
