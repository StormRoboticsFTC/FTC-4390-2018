// this is a copy from Mr. Bross's training package; to be changed as needed
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name="MainAuto", group="Autonomous")
//@Disabled
public class MainAuto extends LinearOpMode {

    // Declare OpMode members.
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null; // no encoder
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null; // no encoder
    private DcMotor intake = null;
    private DcMotor lift = null;
    private ColorSensor colorSensor;
    private ElapsedTime     runtime = new ElapsedTime();

    //Declares variables and constants
    static final double     COUNTS_PER_MOTOR_REV    = 560 ;    //
    static final double     DRIVE_GEAR_REDUCTION    = 1.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.75;
    static final double     TURN_SPEED              = 0.75;

    static final double TURNING_DIAMETER = 18.1; //This and TURNING_CIRCUMFERENCE are used for the turnInPlaceCalc method
    static final double TURNING_CIRCUMFERENCE = TURNING_DIAMETER * 3.1415;

    @Override
    public void runOpMode(){

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).

        leftDrive1 = hardwareMap.get(DcMotor.class, "left_drive1"); //First left drive motor
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2"); //Second left drive motor
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1"); //First right drive motor
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2"); //Second right drive motor
        intake = hardwareMap.get(DcMotor.class, "intake"); //Motor that controls the rubber band intake
        lift = hardwareMap.get(DcMotor.class, "lift1"); //Motor that controls the lift
        colorSensor = hardwareMap.colorSensor.get("color"); //Color sensor for sampling

        //Sets direction of motorsR
        leftDrive1.setDirection(DcMotor.Direction.REVERSE); //Left drive is reversed
        leftDrive2.setDirection(DcMotor.Direction.REVERSE);
        rightDrive1.setDirection(DcMotor.Direction.FORWARD);
        rightDrive2.setDirection(DcMotor.Direction.FORWARD);
        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Resetting Encoders");
        telemetry.update();
        //Resets Encoders
       // rightDrive1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        // Send telemetry message to indicate successful Encoder reset
        //telemetry.addData("Path0", "Starting at %7d",
       //         rightDrive1.getCurrentPosition());
        telemetry.update();
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        colorSensor.enableLed(true);
        // Note: Reverse movement is obtained by setting a negative distance (not speed)

        //Actual program
        //Drops robot
        lift.setPower(-0.75);
        sleep(2500);
        lift.setPower(0.0);
        //Backs away from bar
        msDrive(-0.5, -0.5, 250);
        msDrive(0.5, -0.5, 1225);
        lift.setPower(0.65);
        msDrive(0.75,0.75,1250);
        lift.setPower(0.0);
        intake.setPower(0.75);
        sleep(1500);
        intake.setPower(0.0);

        //Adds telemetry data about path
        telemetry.addData("Path", "Complete");
        telemetry.update();
    }
    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     */
    public void encoderDrive(double speed,
                             double leftInches, double rightInches,
                             double timeoutS) {
        // We have to define targets for both of the motors on each side
        int newLeftTarget;
        int newRightTarget;
        // Ensure that the opmode is still active
        if (opModeIsActive()) {
            // Determine new target position for each motor, and pass to motor controller
            newRightTarget = rightDrive1.getCurrentPosition() + (int)(rightInches * COUNTS_PER_INCH);
            rightDrive1.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            rightDrive1.setTargetPosition(newRightTarget);
            //Set mode for "2" drives if it doesn't work 11/20/18
            // reset the timeout time and start motion.
            runtime.reset();
            leftDrive1.setPower(Math.abs(speed));
            leftDrive2.setPower(Math.abs(speed));
            rightDrive1.setPower(Math.abs(speed));
            rightDrive2.setPower(Math.abs(speed));
            // keep looping while we are still active, and there is time left, and both motors are running.
            while (opModeIsActive() && rightDrive1.isBusy())
            {
                // Display it for the driver.
                //telemetry.addData("Path1",  "Running to %7d ",  newRightTarget);
                //telemetry.addData("Path2",  "Running at %7d ", rightDrive1.getCurrentPosition());
                telemetry.update();
            }
            // Stop all motion;
            leftDrive1.setPower(0);
            leftDrive2.setPower(0);
            rightDrive1.setPower(0);
            rightDrive2.setPower(0);
            // Turn off RUN_TO_POSITION
            rightDrive1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(500);   // optional pause after each move
        }
    }

    //Does the calculations for turning in place
    public double turnInPlaceCalc(int degrees){
        return ((degrees / 360) * TURNING_CIRCUMFERENCE);
    }
    public void msDrive(double leftSpeed, double rightSpeed, long ms) {
        leftDrive1.setPower(leftSpeed);
        leftDrive2.setPower(leftSpeed);
        rightDrive1.setPower(rightSpeed);
        rightDrive2.setPower(rightSpeed);
        sleep(ms);
        leftDrive1.setPower(0.0);
        leftDrive2.setPower(0.0);
        rightDrive1.setPower(0.0);
        rightDrive2.setPower(0.0);
    }
    //Tests for gold color (Sampling)
    public boolean testIfGold() {
        boolean isGold = false;
        int red = colorSensor.red();
        int green = colorSensor.green();
        int blue = colorSensor.blue();
        int alpha = colorSensor.alpha();
        if (green >= (red * 0.25) && green <= (red * 0.75) && blue <=10 && alpha >= green && alpha <= red) {
            isGold = true;
        }
        return isGold;
    }
}