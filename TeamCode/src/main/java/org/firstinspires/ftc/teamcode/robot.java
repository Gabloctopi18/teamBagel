package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

import java.util.List;
public class robot {
    private DcMotor leftFront = null;
    private DcMotor leftBack = null;
    private DcMotor rightFront = null;
    private DcMotor rightBack = null;
    private DcMotor vertSlidesLeft = null;
    private DcMotor vertSlidesRight = null;
    private DcMotor horSlides = null;
    private DcMotor outtakeArm = null;

    private DcMotor[] driveMotors = new DcMotor[]{leftFront, leftBack, rightFront, rightBack};

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
    private LinearOpMode myOpMode;

    public DcMotor leftVertEncoder = null;
    public DcMotor rightVertEncoder = null;
    public DcMotor horEncoder = null;
    public IMU imu;

    private boolean showTelemetry = false;

    public void SimplifiedOdometryRobot(LinearOpMode opmode) {
        myOpMode = opmode;
    }

    public void initialize(boolean showTelemetry){
        leftFront = setupDriveMotor("leftFront", DcMotor.Direction.REVERSE);
        leftBack = setupDriveMotor("leftBack", DcMotor.Direction.REVERSE);
        rightFront = setupDriveMotor("rightFront", DcMotorSimple.Direction.FORWARD);
        rightBack = setupDriveMotor("rightBack", DcMotorSimple.Direction.FORWARD);
        vertSlidesLeft = setupDriveMotor("vertSlidesLeft", DcMotor.Direction.FORWARD);
        vertSlidesRight = setupDriveMotor("vertSlidesRight", DcMotor.Direction.REVERSE);
        horSlides = setupDriveMotor("horSlides", DcMotor.Direction.REVERSE);

        Servo leftIntakeServo = myOpMode.hardwareMap.get(Servo.class, "leftIntakeServo");
        Servo rightIntakeServo = myOpMode.hardwareMap.get(Servo.class, "rightIntakeServo");
        Servo vertClawServo = myOpMode.hardwareMap.get(Servo.class, "vertClawServo");
        Servo horClawServo = myOpMode.hardwareMap.get(Servo.class, "horClawServo");

        imu = myOpMode.hardwareMap.get(IMU.class, "imu");

        leftVertEncoder = myOpMode.hardwareMap.get(DcMotor.class, "leftVertEncoder");
        rightVertEncoder = myOpMode.hardwareMap.get(DcMotor.class, "rightVertEncoder");
        horEncoder = myOpMode.hardwareMap.get(DcMotor.class, "horEncoder");

        //hubs use Auto Bulk caching mode for faster encoder readings
        List<LynxModule> allHubs = myOpMode.hardwareMap.getAll(LynxModule.class);
        for (LynxModule module : allHubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        RevHubOrientationOnRobot.UsbFacingDirection.FORWARD);
        imu.initialize(new IMU.Parameters(orientationOnRobot));

        this.showTelemetry = showTelemetry;
    }

    private DcMotor setupDriveMotor(String deviceName, DcMotor.Direction direction) {
        DcMotor aMotor = myOpMode.hardwareMap.get(DcMotor.class, deviceName);
        aMotor.setDirection(direction);
        aMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);  // Reset Encoders to zero
        aMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        aMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);  // Requires motor encoder cables to be hooked up.
        return aMotor;
    }

    public void pidForward(double speedDamp, double inches, double countsPerInch, double[] constants) {
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
            p = constants[0] * error;
            i = constants[1] * (error + prevError)/2;
            d = constants[2] * (error - prevError)/2;
            power = speedDamp * (p + i + d);
            for (int j = 0; i < 4; i++)
                driveMotors[j].setPower(power);
        }
    }
    public void pidStrafe(double speedDamp, double inches, double countsPerInch, double[] constants) {
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
            p = constants[0] * error;
            i = constants[1] * (error + prevError)/2;
            d = constants[2] * (error - prevError)/2;
            power = speedDamp * (p + i + d);
            leftFront.setPower(power);
            leftBack.setPower(-power);
            rightFront.setPower(-power);
            rightBack.setPower(power);
        }
    }
    public void pidTurn(double speedDamp, double degrees, double[] constants) {
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
            p = constants[0] * error;
            i = constants[1] * (error + prevError)/2;
            d = constants[2] * turnrate;
            power = speedDamp * (p + i + d);
            leftFront.setPower(power);
            leftBack.setPower(power);
            rightFront.setPower(-power);
            rightBack.setPower(-power);
        }
    }

}
