// This OpMode is for the Driver-Controlled Period
/* Controls:
*     Controller 1:
*       Left Stick = Left Drive; Right Stick = Right Drive
*     Controller 2:
*       D-Pad Up/Down = Lift Up/Down
*       Y [Macro] = Lift Completely Up; A [Macro] = Lift Completely Down **WARNING: DO NOT USE Y/A IF LIFT IS NOT EITHER COMPLETELY 
*                                                                                  UP/DOWN TO AVOID GRINDING THE GEARS**
*       Left Trigger = Intake/Outtake Rollers IN; Right Trigger = Intake/Outtake Rollers OUT (Outtake)
*/
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="MainTeleOp", group="Linear Opmode")
//@Disabled
public class MainTeleop extends LinearOpMode {
    // Declare motors/sensors/members
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private ColorSensor colorSensor;

    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
        
        // Initialize the hardware variables. 
        leftDrive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        intake = hardwareMap.get(DcMotor.class, "intake");
        lift = hardwareMap.get(DcMotor.class, "lift1");
        colorSensor = hardwareMap.colorSensor.get("color");
        
        // Sets directions of motors.
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();
        // Enables color sensor LED [not used]
        // colorSensor.enableLed(true);
        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;
            
            // Map power to joysticks
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            // Send power to wheels
            leftDrive1.setPower(leftPower);
            rightDrive1.setPower(rightPower);
            
            // Maps intake/outtake rollers to triggers
            if (gamepad2.left_trigger != 0) {
                intake.setPower(0.5);
            } else if (gamepad2.right_trigger != 0) {
                intake.setPower(-0.5);
            } else {
                intake.setPower(0.0);
            }
            
            // Maps lift to dpad
            if (gamepad2.dpad_up) {
                lift.setPower(-0.6);
            } else if (gamepad2.dpad_down) {
                lift.setPower(0.6);
            } else {
                lift.setPower(0.0);
            }
        
            // Lift Completely Up/Down Macros
            if (gamepad2.y) {
                lift.setPower(-0.6);
                sleep(4400);
                lift.setPower(0.0);
            }
            if (gamepad2.a) {
                lift.setPower(0.6);
                sleep(4400);
                lift.setPower(0.0);
            }
            
            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motor Encoders", "encoder: %d %d", leftDrive1.getCurrentPosition(), rightDrive1.getCurrentPosition());
            //telemetry.addData("ColorSensor", "colo red %d blue %d green %d ", colorSensor.red(), colorSensor.blue(), colorSensor.green());
            telemetry.update();
        }
    }
}


