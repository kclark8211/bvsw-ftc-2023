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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

/*
 * This OpMode illustrates the concept of driving a path based on time.
 * The code is structured as a LinearOpMode
 *
 * The code assumes that you do NOT have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByEncoder;
 *
 *   The desired path in this example is:
 *   - Drive forward for 3 seconds
 *   - Spin right for 1.3 seconds
 *   - Drive Backward for 1 Second
 *
 *  The code is written in a simple form with no optimizations.
 *  However, there are several ways that this type of sequence could be streamlined,
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="ParkBoardSideRed", group="Robot")
public class AutoBoardSideRed extends LinearOpMode {

    /* Declare OpMode members. */
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor elbowMotor = null;
    private Servo leftClaw = null;
    private Servo rightClaw = null;



    private ElapsedTime     runtime = new ElapsedTime();




    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");
        elbowMotor = hardwareMap.get(DcMotor.class, "elbow_motor");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // When run, this OpMode should start both motors driving forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.REVERSE);
        rightFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        rightBackDrive.setDirection(DcMotor.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to run");    //
        telemetry.update();

        // Wait for the game to start (driver presses PLAY)
        waitForStart();

        driveForTimePeriod(0.6, degreesToRadians(180) , 3.5);
        driveForTimePeriod(0.5, degreesToRadians(90), 0.3);
        dropPixel(6000);



        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);
    }

    private void driveForTimePeriod(double magnitude, double direction, double timeInSeconds)
    {
        double currentTime = runtime.seconds();
        double motorsA = calculateMotorPowerA(direction, magnitude);
        double motorsB = calculateMotorPowerB(direction, magnitude);
        while(runtime.seconds() <= currentTime + timeInSeconds)
        {

            rightFrontDrive.setPower(motorsA);
            leftBackDrive.setPower(motorsA);
            rightBackDrive.setPower(motorsB);
            leftFrontDrive.setPower(motorsB);
        }
        rightFrontDrive.setPower(0);
        leftBackDrive.setPower(0);
        rightBackDrive.setPower(0);
        leftFrontDrive.setPower(0);
    }

    private double calculateMotorPowerA(double theta, double mag)
    {
        return ((Math.sin(theta) + Math.cos(theta))/2) * mag;
    }
    //used for rightfront and leftback

    private double calculateMotorPowerB(double theta, double mag)
    {
        return ((Math.sin(theta) - Math.cos(theta))/2) * mag;
    }

    private double degreesToRadians(double degrees)
    {
        return degrees * (Math.PI / 180);
    }

    private void dropPixel(int elbowTarget)
    {

        while(elbowTarget != elbowMotor.getCurrentPosition()) {
            if (elbowTarget - 100 >= elbowMotor.getCurrentPosition()) {
                elbowMotor.setPower(0.5);
            } else if (elbowTarget + 100 <= elbowMotor.getCurrentPosition()) {
                elbowMotor.setPower(-0.5);
            } else if (elbowTarget >= elbowMotor.getCurrentPosition()) {
                //elbowMotor.setPower(((elbowTarget- elbowMotor.getCurrentPosition() / 100) * -0.5));
                elbowMotor.setPower(0.1);
            } else if (elbowTarget <= elbowMotor.getCurrentPosition()) {
                //elbowMotor.setPower(((elbowTarget- elbowMotor.getCurrentPosition() / 100) * 0.5));
                elbowMotor.setPower(-0.1);
            }
        }
        double time = runtime.seconds();
        while(runtime.seconds() <= time + 1) {
            leftClaw.setPosition(0.2);
            rightClaw.setPosition(0.2);
        }
        while(runtime.seconds() <= time + 2) {
            leftClaw.setPosition(0);
            rightClaw.setPosition(0);
        }
        while(0 != elbowMotor.getCurrentPosition()) {
            if (elbowTarget - 100 >= elbowMotor.getCurrentPosition()) {
                elbowMotor.setPower(0.5);
            } else if (elbowTarget + 100 <= elbowMotor.getCurrentPosition()) {
                elbowMotor.setPower(-0.5);
            } else if (elbowTarget >= elbowMotor.getCurrentPosition()) {
                //elbowMotor.setPower(((elbowTarget- elbowMotor.getCurrentPosition() / 100) * -0.5));
                elbowMotor.setPower(0.1);
            } else if (elbowTarget <= elbowMotor.getCurrentPosition()) {
                //elbowMotor.setPower(((elbowTarget- elbowMotor.getCurrentPosition() / 100) * 0.5));
                elbowMotor.setPower(-0.1);
            }
        }
    }
    //used for leftfront and rightback
}


