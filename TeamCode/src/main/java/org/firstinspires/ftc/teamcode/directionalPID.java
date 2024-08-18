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

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

/*
 * This OpMode illustrates the concept of driving a path based on encoder counts.
 * The code is structured as a LinearOpMode
 *
 * The code REQUIRES that you DO have encoders on the wheels,
 *   otherwise you would use: RobotAutoDriveByTime;
 *
 *  This code ALSO requires that the drive Motors have been configured such that a positive
 *  power command moves them forward, and causes the encoders to count UP.
 *
 *   The desired path in this example is:
 *   - Drive forward for 48 inches
 *   - Spin right for 12 Inches
 *   - Drive Backward for 24 inches
 *   - Stop and close the claw.
 *
 *  The code is written using a method called: encoderDrive(speed, leftInches, rightInches, timeoutS)
 *  that performs the actual movement.
 *  This method assumes that each movement is relative to the last stopping place.
 *  There are other ways to perform encoder based moves, but this method is probably the simplest.
 *  This code uses the RUN_TO_POSITION mode to enable the Motor controllers to generate the run profile
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@Autonomous(name="Autonomous With Encoders Test", group="Robot")
public class directionalPID extends LinearOpMode {

    /* Declare OpMode members. */
    public DcMotor leftFront = null;
    public DcMotor leftBack = null;
    public DcMotor rightFront = null;
    public DcMotor rightBack = null;
    public DcMotor leftVertEncoder = null;
    public DcMotor rightVertEncoder = null;
    public DcMotor horEncoder = null;
    public IMU imu;
    private LinearOpMode myOpMode;
    double[] forwardConstants = new double[]{1, 0.01, 0.5}; // {kp, ki, kd}
    double[] strafeConstants = new double[]{1, 0.01, 0.5};
    double[] turnConstants = new double[]{0, 0.01, 0.5};

    static final double countsPerRev = 2000; //counts per revolution for gobilda odometry
    static final double odoWheelDiameter = 1.88976; //diameter in inches
    static final double pi = 3.1415926535;
    static final double countsPerInch = (countsPerRev)/(odoWheelDiameter * pi);

    DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

    @Override
    public void runOpMode() {

        // Initialize the drive system variables.
        leftFront = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack = hardwareMap.get(DcMotor.class, "rightBack");

        leftVertEncoder = myOpMode.hardwareMap.get(DcMotor.class, "leftVertEncoder");
        rightVertEncoder = myOpMode.hardwareMap.get(DcMotor.class, "rightVertEncoder");
        horEncoder = myOpMode.hardwareMap.get(DcMotor.class, "horEncoder");

        imu = hardwareMap.get(IMU.class, "imu");
        // Adjust the orientation parameters to match your robot
        IMU.Parameters parameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(parameters);

        leftFront.setDirection(DcMotor.Direction.REVERSE);
        leftBack.setDirection(DcMotor.Direction.REVERSE);

        for (int i = 0; i < 4; i++){
            motors[i].setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }



        waitForStart();

        pidForward(1.0, 12.0, countsPerInch, forwardConstants[0], forwardConstants[1], forwardConstants[2]);
        pidStrafe(1.0, 12.0, countsPerInch, strafeConstants[0], strafeConstants[1], strafeConstants[2]);
        pidTurn(1.0, 90, turnConstants[0], turnConstants[1], turnConstants[2]);

        telemetry.addData("Path", "Complete");
        telemetry.update();
        sleep(1000);  // pause to display final telemetry message.
    }

    /*
     *  Method to perform a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the OpMode running.
     */
    public void pidForward(double speedDamp, double inches, double countsPerInch, double kp, double ki, double kd) {
        double p;
        double i;
        double d;
        double power;
        double prevError;
        double target = countsPerInch * inches;
        double error = target - ((rightVertEncoder.getCurrentPosition() + leftVertEncoder.getCurrentPosition())/2.0);
        while (error != 0.0){
            prevError = error;
            error = target - ((rightVertEncoder.getCurrentPosition() + leftVertEncoder.getCurrentPosition())/2.0);
            p = kp * error;
            i = ki * (error + prevError)/2;
            d = kd * (error - prevError)/2;
            power = speedDamp * (p + i + d);
            for (int j = 0; i < 4; i++)
                motors[j].setPower(power);
        }
    }
    public void pidStrafe(double speedDamp, double inches, double countsPerInch, double kp, double ki, double kd) {
        double p;
        double i;
        double d;
        double power;
        double prevError;
        double target = countsPerInch * inches;
        double error = target - horEncoder.getCurrentPosition();
        while (error != 0.0){
            prevError = error;
            error = target - horEncoder.getCurrentPosition();
            p = kp * error;
            i = ki * (error + prevError)/2;
            d = kd * (error - prevError)/2;
            power = speedDamp * (p + i + d);
            leftFront.setPower(power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(power);
        }
    }
    public void pidTurn(double speedDamp, double degrees, double kp, double ki, double kd) {
        double p;
        double i;
        double d;
        double power;
        double prevError;
        double target = degrees;
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        AngularVelocity angularVelocity = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        double error = target - orientation.getYaw(AngleUnit.DEGREES);
        double turnrate = angularVelocity.zRotationRate;
        while (error != 0.0){
            prevError = error;
            error = target - (horEncoder.getCurrentPosition()/2.0);
            p = kp * error;
            i = ki * (error + prevError)/2;
            d = kd * turnrate;
            power = speedDamp * (p + i + d);
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }
    }
}