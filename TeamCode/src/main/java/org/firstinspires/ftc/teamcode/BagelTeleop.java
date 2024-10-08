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
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


/*
 * This file contains an minimal example of a Linear "OpMode". An OpMode is a 'program' that runs in either
 * the autonomous or the teleop period of an FTC match. The names of OpModes appear on the menu
 * of the FTC Driver Station. When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all linear OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */

@TeleOp(name="bagelTeleop", group="Linear OpMode")
//@Disabled
public class BagelTeleop extends LinearOpMode {

    //Teleop Variables
    double y = 0;
    double x = 0;
    double rx = 0;
    double denom = 0;
    double leftFrontPower;
    double leftBackPower;
    double rightFrontPower;
    double rightBackPower;

    // Declare OpMode members.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor vertSlidesLeft = null;
    private DcMotor vertSlidesRight = null;
    private DcMotor horSlides = null;
    private DcMotor outtakeArm = null;

    boolean outtakeUsingEncoders = true;
    boolean vertClawOpen = false;
    boolean horClawOpen = false;
    boolean outtakeArmDeposit = false;
    boolean intakeArmPickup = false;

    //NEED TO TEST FOR THESE VALUES
    int depositPosition = 100;
    int pickupPosition = -100;
    double leftIntakePickupPos = 1;
    double leftIntakeDropPos = 0;
    double rightIntakePickupPos = 1;
    double rightIntakeDropPos = 0;
    double vertClawClosePos = 1;
    double vertClawOpenPos = 0;
    double horClawClosePos = 1;
    double horClawOpenPos = 0;



    @Override
    public void runOpMode() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        // Initialize the hardware variables. Note that the strings used here as parameters
        // to 'get' must correspond to the names assigned during the robot configuration
        // step (using the FTC Robot Controller app on the phone).
        leftFront  = hardwareMap.get(DcMotor.class, "leftFront");
        rightFront = hardwareMap.get(DcMotor.class, "rightFront");
        leftBack  = hardwareMap.get(DcMotor.class, "leftBack");
        rightBack  = hardwareMap.get(DcMotor.class, "rightBack");
        vertSlidesRight  = hardwareMap.get(DcMotor.class, "vertSlidesRight");
        vertSlidesLeft = hardwareMap.get(DcMotor.class, "vertSlidesLeft");
        horSlides = hardwareMap.get(DcMotor.class, "horSlidesRight");
        outtakeArm = hardwareMap.get(DcMotor.class, "outtakeArm");

        Servo leftIntakeServo = hardwareMap.get(Servo.class, "leftIntakeServo");
        Servo rightIntakeServo = hardwareMap.get(Servo.class, "rightIntakeServo");
        Servo vertClawServo = hardwareMap.get(Servo.class, "vertClawServo");
        Servo horClawServo = hardwareMap.get(Servo.class, "horClawServo");


        // To drive forward, most robots need the motor on one side to be reversed, because the axles point in opposite directions.
        // Pushing the left stick forward MUST make robot go forward. So adjust these two lines based on your first test drive.
        // Note: The settings here assume direct drive on left and right wheels.  Gear Reduction or 90 Deg drives may require direction flips
        rightFront.setDirection(DcMotor.Direction.REVERSE);
        rightBack.setDirection(DcMotor.Direction.REVERSE);
        vertSlidesRight.setDirection(DcMotorSimple.Direction.REVERSE);
        horSlides.setDirection(DcMotorSimple.Direction.REVERSE);

        DcMotor[] motors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack, vertSlidesLeft, vertSlidesRight, horSlides, outtakeArm};

        outtakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        outtakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        for (int i = 0; i < 8; i++){
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
        runtime.reset();

        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {
            //drivetrain code
            y = -gamepad1.left_stick_y; // Remember, Y stick value is reversed
            x = gamepad1.left_stick_x * 1.1; // Counteract imperfect strafing
            rx = gamepad1.right_stick_x;
            denom = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1); //use denom to keep the drive ratios (dont get it clipped)

            leftFrontPower = (y + x + rx) / denom;
            leftBackPower = (y - x + rx) / denom;
            rightFrontPower = (y - x - rx) / denom;
            rightBackPower = (y + x - rx) / denom;
            leftFront.setPower(leftFrontPower);
            leftBack.setPower(leftBackPower);
            rightFront.setPower(rightFrontPower);
            rightBack.setPower(rightBackPower);


            vertSlidesLeft.setPower(-gamepad2.right_stick_y);
            vertSlidesRight.setPower(-gamepad2.right_stick_y);
            horSlides.setPower(gamepad2.left_trigger - gamepad2.right_trigger);

            leftIntakeServo.setPosition(0.5 + 0.5*gamepad2.left_stick_y);
            rightIntakeServo.setPosition(0.5 - 0.5*gamepad2.left_stick_y);

            if (gamepad2.left_bumper)
                horClawServo.setPosition(horClawOpen ? horClawClosePos : horClawOpenPos);
            if (gamepad2.right_bumper)
                vertClawServo.setPosition(vertClawOpen ? vertClawClosePos : vertClawOpenPos);
            if (gamepad2.a)
                outtakeArm.setTargetPosition(outtakeArmDeposit ? depositPosition : pickupPosition);
            if (gamepad2.b){
                leftIntakeServo.setPosition(intakeArmPickup ? leftIntakeDropPos : leftIntakePickupPos);
                rightIntakeServo.setPosition(intakeArmPickup ? rightIntakeDropPos : rightIntakePickupPos);
            }
            if (gamepad2.dpad_up)
                outtakeArm.setTargetPosition(outtakeArm.getCurrentPosition() + 10);
            if (gamepad2.dpad_down)
                outtakeArm.setTargetPosition(outtakeArm.getCurrentPosition() - 10);

            //reset the outtake motor in case the encoders go haywire
            if (gamepad2.back){ //use left bumper to activate manual mode
                outtakeArm.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                outtakeArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
                outtakeArm.setPower(gamepad2.right_trigger - gamepad2.left_trigger);
                outtakeUsingEncoders = false;
                if (gamepad2.options){ //reset with right bumper only works when already in manual mode
                    if (outtakeArmDeposit)
                        depositPosition = outtakeArm.getCurrentPosition();
                    else
                        pickupPosition = outtakeArm.getCurrentPosition();
                    outtakeArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    outtakeArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    outtakeArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    outtakeUsingEncoders = true;
                }
            }

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Motors", "leftFront (%.2f), rightFront (%.2f)", leftFrontPower, rightFrontPower);
            telemetry.addData("Motors", "leftBack (%.2f), rightBack (%.2f)", leftBackPower, rightBackPower);
            telemetry.addData("Outtake Motor", "position (%d), mode (%s)", outtakeArm.getCurrentPosition(), outtakeUsingEncoders?"using encoders":"manual movement");
            telemetry.update();
        }
    }
}
