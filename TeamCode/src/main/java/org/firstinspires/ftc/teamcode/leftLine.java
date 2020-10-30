
package org.firstinspires.ftc.teamcode;import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name="Drive Forward", group="")
public class leftLine extends LinearOpMode {

    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftbackDrive = null;
    private DcMotor rightbackDrive = null;
    private DcMotor leftfrontDrive = null;
    private DcMotor rightfrontDrive = null;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        leftbackDrive  = hardwareMap.get(DcMotor.class, "leftbackdrive");
        rightbackDrive = hardwareMap.get(DcMotor.class, "rightbackdrive");
        leftfrontDrive  = hardwareMap.get(DcMotor.class, "leftfrontdrive");
        rightfrontDrive = hardwareMap.get(DcMotor.class, "rightfrontdrive");

        leftbackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightbackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftfrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightfrontDrive.setDirection(DcMotor.Direction.FORWARD);

        waitForStart();

        telemetry.addData("Mode", "running");
        telemetry.update();

        // set motors to 25% power.
        sleep(25000);

        leftbackDrive.setPower(0.25);
        rightbackDrive.setPower(0.25);
        leftfrontDrive.setPower(0.25);
        rightfrontDrive.setPower(0.25);

        sleep(2000);        // wait for 2 seconds.

        // set motor power to zero to stop motors.

        leftbackDrive.setPower(0);
        rightbackDrive.setPower(0);
        leftfrontDrive.setPower(0);
        rightfrontDrive.setPower(0);
    }
}