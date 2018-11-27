/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/**
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When an selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studios to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */

@TeleOp(name="MainTeleOp", group="Linear Opmode")
//@Disabled
public class MainTeleop extends LinearOpMode {

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftDrive1 = null;
    private DcMotor leftDrive2 = null;
    private DcMotor rightDrive1 = null;
    private DcMotor rightDrive2 = null;
    private DcMotor intake = null;
    private DcMotor lift = null;
    private ColorSensor colorSensor;
    private Servo outtake = null;


    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftDrive1 = hardwareMap.get(DcMotor.class, "left_drive1");
        leftDrive2 = hardwareMap.get(DcMotor.class, "left_drive2");
        rightDrive1 = hardwareMap.get(DcMotor.class, "right_drive1");
        rightDrive2 = hardwareMap.get(DcMotor.class, "right_drive2");
 //       intake = hardwareMap.get(DcMotor.class, "intake");
   //     lift = hardwareMap.get(DcMotor.class, "lift1");
     //   colorSensor = hardwareMap.colorSensor.get("color");

        // Most robots need the motor on one side to be reversed to drive forward
        // Reverse the motor that runs backwards when connected directly to the battery
        rightDrive1.setDirection(DcMotor.Direction.REVERSE);
        rightDrive2.setDirection(DcMotor.Direction.REVERSE);
        leftDrive1.setDirection(DcMotor.Direction.FORWARD);
        leftDrive2.setDirection(DcMotor.Direction.FORWARD);

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        //Enables color sensor LED
       // colorSensor.enableLed(true);

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            // Setup a variable for each drive wheel to save power level for telemetry
            double leftPower;
            double rightPower;

            // Choose to drive using either Tank Mode, or POV Mode
            // Comment out the method that's not used.  The default below is POV.

            // POV Mode uses left stick to go forward, and right stick to turn.
            //
            // - This uses basic math to combine motions and is easier to drive straight.
            //double drive = -gamepad1.left_stick_y;
            //double turn  =  gamepad1.right_stick_x;
            //leftPower    = Range.clip(drive + turn, -1.0, 1.0) ;
            //rightPower   = Range.clip(drive - turn, -1.0, 1.0) ;

            // Tank Mode uses one stick to control each wheel.
            // - This requires no math, but it is hard to drive forward slowly and keep straight.
            leftPower = gamepad1.left_stick_y;
            rightPower = gamepad1.right_stick_y;

            // Send calculated power to wheels
            leftDrive1.setPower(leftPower);
            leftDrive2.setPower(leftPower);
            rightDrive1.setPower(rightPower);
            rightDrive2.setPower(rightPower);


            //controls the intake: option 1
            if (gamepad2.left_trigger != 0) {
                intake.setPower(0.5);
            }
            else if (gamepad2.right_trigger != 0){
                intake.setPower(-0.5);
            } else {
                intake.setPower(0.0);
            }



            // this is an on and off control, where separate
            // an alternate would be to use the trigger to turn it on and use an else to turn it off
            // when the trigger is NOT pressed

            //controls the lift
           if (gamepad2.dpad_up) {
                lift.setPower(0.6);
            } else if (gamepad2.dpad_down) {
                lift.setPower(-0.6);
            } else {
                lift.setPower(0.0);
            }


            // Show the elapsed game time and wheel power.
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "left (%.2f), right (%.2f)", leftPower, rightPower);
            telemetry.addData("Motor Encoders", "left1 %d left2 %d right1 %d right2 %d", leftDrive1.getCurrentPosition(), leftDrive2.getCurrentPosition(), rightDrive1.getCurrentPosition(), rightDrive2.getCurrentPosition());
            //telemetry.addData("ColorSensor", "colo red %d blue %d green %d ", colorSensor.red(), colorSensor.blue(), colorSensor.green());
            sleep(150);
            telemetry.update();
        }
    }
}