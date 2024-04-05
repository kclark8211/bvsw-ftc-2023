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

// Truett Van Slyke 2024
package org.firstinspires.ftc.teamcode;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@TeleOp(name="MainDrive2024", group="Linear OpMode")
public class MainDrive2024 extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor frontLeftDrive = null;
    private DcMotor backLeftDrive = null;
    private DcMotor frontRightDrive = null;
    private DcMotor backRightDrive = null;

    private double targetOrientation = 0;

    private boolean[] bumperStates = {false, false};

    BNO055IMU imu;
    Orientation angles;

    @Override
    public void runOpMode() {
        // Initialize IMU
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        frontLeftDrive  = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive  = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // MOTOR DRIVE DIRECTIONS
        // Fit for each robot. When fully powered, all wheels should spin
        // in the same direction (forward).
        frontLeftDrive.setDirection(DcMotor.Direction.FORWARD);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("STATUS", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        // Loop while opmode is enabled
        while (opModeIsActive()) {
            checkBumpers();
            targetOrientation = formatAngle(targetOrientation);

            double currentOrientation = imu.getAngularOrientation().firstAngle;
            currentOrientation = formatAngle(currentOrientation);

            double orientationCorrect = leastAngularCorrection(currentOrientation, targetOrientation);
            orientationCorrect /= 90;
            orientationCorrect = restrict(orientationCorrect, -1, 1);
            if (Math.abs(orientationCorrect) < 0.02) {
                orientationCorrect = 0;
            }

            double[] drivePolar = leftStickPolar();
            double driveTheta = drivePolar[0];
            double driveMag   = drivePolar[1];

            double[] motorPowers = calcMotorPowersFromPolar(driveTheta, driveMag, orientationCorrect);

            frontLeftDrive.setPower(motorPowers[0]);
            frontRightDrive.setPower(motorPowers[1]);
            backLeftDrive.setPower(motorPowers[2]);
            backRightDrive.setPower(motorPowers[3]);

            telemetry.addData("STATUS", "Running");
            telemetry.addData("ORIENTATION CORRECT", orientationCorrect);
            telemetry.addData("DRIVE THETA", driveTheta);
            telemetry.addData("DRIVE MAG", driveMag);
            telemetry.addData("IMU ANGLES", imu.getAngularOrientation().firstAngle);
            telemetry.addData("TARGET ORIENTATION", targetOrientation);
            telemetry.update();
        }
    }

    private double formatAngle(double angle) {
        // Restrict angle to [-180, 180]
        while (Math.abs(angle) > 180) {
            angle -= (Math.abs(angle) / angle) * 360; // Subtracts 180 * sign of number
        }

        return angle;
    }
    private double leastAngularCorrection(double angle, double targetAngle) {
        angle = formatAngle(angle);
        targetAngle = formatAngle(targetAngle);

        double correction = targetAngle - angle;
        double leastCorrection = correction;

        angle += 360;
        correction = targetAngle - angle;
        if (Math.abs(correction) < Math.abs(leastCorrection)) {
            leastCorrection = correction;
        }

        angle -= 360 * 2;
        correction = targetAngle - angle;
        if (Math.abs(correction) < Math.abs(leastCorrection)) {
            leastCorrection = correction;
        }

        return leastCorrection;
    }

    private double restrict(double num, double min, double max) {
        return Math.max(Math.min(max, num), min);
    }

    private double[] rectToPolar(double x, double y) {
        double theta = Math.atan2(y, x);
        double mag = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));

        return new double[] {theta, mag};
    }

    // Functions to easily retrieve controller input
    private double[] leftStickPolar() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        return rectToPolar(x, y);
    }

    private double[] leftStickRect() {
        double x = gamepad1.left_stick_x;
        double y = gamepad1.left_stick_y;

        return new double[] {x, y};
    }

    private double [] rightStickPolar() {
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        return rectToPolar(x, y);
    }

    private double[] rightStickRect() {
        double x = gamepad1.right_stick_x;
        double y = gamepad1.right_stick_y;

        return new double[] {x, y};
    }

    // Function to calculate motor powers needed to drive in given direction
    private double[] calcMotorPowersFromPolar(double theta, double mag, double turn) {
        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin), Math.abs(cos));

        double leftFront  = mag * cos/max + turn;
        double rightFront = mag * sin/max - turn;
        double leftRear   = mag * sin/max + turn;
        double rightRear  = mag * cos/max - turn;

        if ((mag + Math.abs(turn)) > 1) {
            leftFront  /= mag + turn;
            rightFront /= mag + turn;
            leftRear   /= mag + turn;
            rightRear  /= mag + turn;
        }

        return new double[] {leftFront, rightFront, leftRear, rightRear};
    }

    private double[] calcMotorPowersFromRect(double x, double y, double turn) {
        // Convert to polar so we can reuse calcMotorPowersFromPolar()
        double[] polarCoords = rectToPolar(x, y);
        double theta = polarCoords[0];
        double mag   = polarCoords[1];

        double[] motorPowers = calcMotorPowersFromPolar(theta, mag, turn);
        return motorPowers;
    }

    private void checkBumpers() {
        if (gamepad1.left_bumper) {
            if (bumperStates[0]) {
                // Bumper held
            } else {
                // Bumper new push
                bumperStates[0] = true;
                targetOrientation += 90;
            }
        } else {
            // Bumper released
            bumperStates[0] = false;
        }
        if (gamepad1.right_bumper) {
            if (bumperStates[1]) {
                // Bumper held
            } else {
                // Bumper new push
                bumperStates[1] = true;
                targetOrientation -= 90;}
        } else {
            // Bumper released
            bumperStates[1] = false;
        }
    }
}