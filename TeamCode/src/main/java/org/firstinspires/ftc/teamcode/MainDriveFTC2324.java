/* Copyright (c) 2021 FIRST. All rights reserved.
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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.opencv.core.Mat;

@TeleOp(name="mainDrive", group="Linear OpMode")
public class MainDriveFTC2324 extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFrontDrive = null;
    private DcMotor leftBackDrive = null;
    private DcMotor rightFrontDrive = null;
    private DcMotor rightBackDrive = null;
    private DcMotor elbowMotor = null;

    private double theta;
    private double magnitude;

    private double getX;
    private double getY;

    private double spinLF;

    private double spinRF;

    private double spinLB;

    private double spinRB;

    private double turnDirection;

    private double heading;

    private double speed = 1;

    private double driveAngle;

    private int elbowTarget = 0;
    private double wristTarget = 0;

    private Servo wristServo = null;

    private Servo leftClaw = null;

    private Servo rightClaw = null;

    private int pickUpPos = 8850;

    private boolean dropActive=false;




    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);


        leftFrontDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        leftBackDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        rightFrontDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        rightBackDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        elbowMotor = hardwareMap.get(DcMotor.class, "elbow_motor");
        wristServo = hardwareMap.get(Servo.class, "wrist_servo");
        leftClaw = hardwareMap.get(Servo.class, "left_claw");
        rightClaw = hardwareMap.get(Servo.class, "right_claw");

        leftFrontDrive.setDirection(DcMotor.Direction.FORWARD);
        leftBackDrive.setDirection(DcMotor.Direction.FORWARD);
        rightFrontDrive.setDirection(DcMotor.Direction.REVERSE);
        rightBackDrive.setDirection(DcMotor.Direction.REVERSE);
        elbowMotor.setDirection(DcMotorSimple.Direction.FORWARD);
        leftClaw.setDirection(Servo.Direction.REVERSE);
        wristServo.resetDeviceConfigurationForOpMode();

        elbowMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();


        while (opModeIsActive()) {
            angles = imu.getAngularOrientation();
            getX = gamepad1.left_stick_x;
            getY = gamepad1.left_stick_y;
            driveAngle = getAngle(getX, getY);
            magnitude = getMag(getX, getY);

            heading = angles.firstAngle * (Math.PI / 180);
            theta = driveAngle + heading;
            if (theta > Math.PI)
            {
                theta = theta - (Math.PI * 2);
            }

            if (theta < -1 * Math.PI)
            {
                theta = theta + (Math.PI * 2);
            }

            if (gamepad1.right_bumper)
            {
                turnDirection -= 1;
            }
            if (gamepad1.left_bumper)
            {
                turnDirection += 1;
            }

            double[] motorPowers = getMoterPowers(theta, magnitude, turnDirection);
            leftFrontDrive.setPower(motorPowers[0]);
            rightFrontDrive.setPower(motorPowers[1]);
            leftBackDrive.setPower(motorPowers[2]);
            rightBackDrive.setPower(motorPowers[3]);

            turnDirection = 0;


            // ---------------------------
            // Move elbow with joystick y
            elbowMotor.setPower(gamepad1.right_stick_y / 2);
            if (gamepad1.right_stick_y != 0)
            {
                // If joystick is being used, set target position to current position
                elbowTarget = elbowMotor.getCurrentPosition() ;
            }


            // Elbow adjust to target position
            double elbowPosition = elbowMotor.getCurrentPosition();

            double elbowError = elbowTarget - elbowPosition;
            double elbowAdjust = elbowError;

            if (Math.abs(elbowAdjust) >= 100) {
                // Large arm error - Restrict adjust to [-0.5, 0.5]
                elbowAdjust = Math.min(Math.max(elbowAdjust, -0.5), 0.5);
            } else if (Math.abs(elbowAdjust) > 0) {
                // Small arm error - Restrict adjust to [-0.1, 0.1]
                elbowAdjust = Math.min(Math.max(elbowAdjust, -0.1), 0.1);
            }

            if (Math.abs(elbowAdjust) >= 0.1) {
                elbowMotor.setPower(elbowAdjust);
            }

//            // Legacy arm adjust
//            if (elbowTarget - 100 >= elbowPosition)
//            {
//                elbowMotor.setPower(0.5);
//            }
//            else if (elbowTarget + 100 <= elbowPosition)
//            {
//                elbowMotor.setPower(-0.5);
//            }
//            else if (elbowTarget >= elbowPosition)
//            {
//                elbowMotor.setPower(0.1);
//            }
//            else if (elbowTarget <= elbowTarget)
//            {
//                elbowMotor.setPower(-0.1);
//            }

//            // Elbow target control
//            if (gamepad1.right_stick_y != 0) {
//                elbowTarget -= (int)(gamepad1.right_stick_y * 20);
//            }


            // Wrist Control
            double wristControl = gamepad1.left_trigger - gamepad1.right_trigger;
            wristTarget += wristControl;

            wristServo.setPosition(wristTarget);

            // Claw Control
            if(gamepad1.dpad_down)
            {
                leftClaw.setPosition(0);
                rightClaw.setPosition(0);
            }
            if (gamepad1.dpad_up)
            {
                leftClaw.setPosition(0.15);
                rightClaw.setPosition(0.15);
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Angle", "stickAngle " + getAngle(getX, getY));
            telemetry.addData("Magnitude", "stickMag " + getMag(getX, getY));
            telemetry.addData("xStick", "x: " + (getX));
            telemetry.addData("yStick", "y: " + (getY));
            telemetry.addData("aPower", "a" + motorA(theta, magnitude));
            telemetry.addData("bPower", "b" + motorB(theta, magnitude));
            telemetry.addData("Heading", formatAngle(angles.angleUnit, angles.firstAngle));
            telemetry.addData("isX ", gamepad1.x);
/*
            telemetry.addData("motorPosition", elbowMotor.getCurrentPosition());
            telemetry.addData("Target", elbowTarget);
            //telemetry.addData("servoPosition", servo.getPosition());
            telemetry.addData("leftPosition", leftClaw.getPosition());
            //telemetry.addData("rightPosition", rightClaw.getPosition());
            telemetry.addData("Wrist Angle", servo.getPosition());
*/
            telemetry.update();
        }

    }

    private double getAngle(double x, double y) {
        return Math.atan2(y,x);
    }

    private double getMag(double x, double y) {
        return Math.sqrt((x*x)+(y*y));
    }

    private double motorA(double theta, double mag)
    {
       return ((Math.sin(theta) + Math.cos(theta))/2) * mag;
    }

    private double motorB(double theta, double mag)
    {
        return ((Math.sin(theta) - Math.cos(theta))/2) * mag;
    }

    String formatAngle(AngleUnit angleUnit, double angle) {
        return String.format("%.2f", AngleUnit.DEGREES.fromUnit(angleUnit, angle));
    }



    private double[] getMoterPowers(double theta, double mag, double turn) {
        double sin = Math.sin(theta - Math.PI/4); // Unchecked
        double cos = Math.cos(theta - Math.PI/4); // Unchecked
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront = mag * sin/max + turn;  //reversed all 4, sin <-> cos
        double rightFront = mag * cos/max - turn;
        double leftRear = mag * cos/max + turn;
        double rightRear = mag * sin/max  - turn;

        if ((mag + Math.abs(turn)) > 1) {
            leftFront /= mag + turn;
            rightFront /= mag + turn;
            leftRear /= mag + turn;
            rightRear /= mag + turn;
        }

        return new double[]{leftFront, rightFront, leftRear, rightRear};
    }

}

