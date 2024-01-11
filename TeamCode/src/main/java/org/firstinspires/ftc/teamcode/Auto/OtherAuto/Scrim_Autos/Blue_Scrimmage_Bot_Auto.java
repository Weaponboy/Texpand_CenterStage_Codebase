package org.firstinspires.ftc.teamcode.Auto.OtherAuto.Scrim_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous
@Disabled
public class Blue_Scrimmage_Bot_Auto extends LinearOpMode {

    DcMotor LF;
    DcMotor RF;
    DcMotor Povit2;
    Servo PivotLeft;

    public WebcamName frontCam;

    public VisionPortal portal;

    static final double     COUNTS_PER_MOTOR_REV    = 480;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCM   = 9;
    static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCM * 3.1415);

    public BNO055IMU imu = null;

    Servo PivotServo;

    propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.blue);

    @Override
    public void runOpMode() throws InterruptedException {

        RF = hardwareMap.get(DcMotor.class,"RightDrive");
        LF = hardwareMap.get(DcMotor.class,"LeftDrive");
        Povit2 = hardwareMap.get(DcMotor.class, "Pivot");
        PivotLeft = hardwareMap.get(Servo.class, "Gripper");
        PivotServo = hardwareMap.get(Servo.class,"PivotServo");

        Povit2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Povit2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        frontCam = hardwareMap.get(WebcamName.class, "frontCam");

        PivotLeft.setPosition(1);
        PivotServo.setPosition(0.5);

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);

        waitForStart();

//        while (opModeIsActive()){
//            double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;
//
//            double ConvertedHeading;
//
//            if (heading <= 0) {
//                ConvertedHeading = (360 + heading);
//            } else {
//                ConvertedHeading = (0 + heading);
//            }
//
//            double rotdist = (90 - ConvertedHeading);
//
//            if (rotdist < -180) {
//                rotdist = (360 + rotdist);
//            } else if (rotdist > 360) {
//                rotdist = (rotdist - 360);
//            }
//
//            telemetry.addData("heading", 360 - ConvertedHeading);
//            telemetry.addData("rotdist", rotdist);
//            telemetry.update();
//        }

        //start code#
//        propPos = 4;

        if (propPos == 1) {

            encoderDrive(0.4, 40);
            turnToHeadingWithImu(imu, 350, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 340, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 330, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 320, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 310, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 300, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 290, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 280, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 270, this);

            encoderDrive(0.4, -10);

            turnToHeadingWithImu(imu, 215, this);

            encoderDrive(0.4, 22);

            turnToHeadingWithImu(imu, 270, this);

            encoderDrive(0.4, 80);

            sleep(200);

            Povit2.setTargetPosition(1200);

            Povit2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Povit2.setPower(0.7);

            PivotServo.setPosition(0.2);

            while (Povit2.isBusy()) {}

            sleep(1000);

            PivotLeft.setPosition(0.4);

            sleep(1500);

            Povit2.setTargetPosition(0);

            Povit2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Povit2.setPower(-0.7);

            sleep(1000);

        } else if (propPos == 2) {

            encoderDrive(0.4, 80);
            encoderDrive(0.4, -16);
            turnToHeadingWithImu(imu, 270, this);

            sleep(500);

            encoderDrive(0.4, 96);

            sleep(200);

            Povit2.setTargetPosition(1000);

            Povit2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Povit2.setPower(0.7);

            PivotServo.setPosition(0.2);

            while (Povit2.isBusy()) {}

            sleep(1000);

            PivotLeft.setPosition(0.4);

            sleep(1500);

            Povit2.setTargetPosition(0);

            Povit2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Povit2.setPower(-0.7);

            sleep(1000);

        } else if (propPos == 3) {

            encoderDrive(0.4, 40);
            turnToHeadingWithImu(imu, 10, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 20, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 30, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 40, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 50, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 60, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 70, this);
            encoderDrive(0.4, 5);
            turnToHeadingWithImu(imu, 80, this);
            encoderDrive(0.4, 10);
            turnToHeadingWithImu(imu, 90, this);

            encoderDrive(0.4, -12);

            turnToHeadingWithImu(imu, 0, this);

            turnToHeadingWithImu(imu, 280, this);

            encoderDrive(0.4, 105);

            turnToHeadingWithImu(imu, 275, this);
            sleep(200);

            Povit2.setTargetPosition(1200);

            Povit2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Povit2.setPower(0.7);

            PivotServo.setPosition(0.2);

            while (Povit2.isBusy()) {

            }

            sleep(1000);

            PivotLeft.setPosition(0.4);

            sleep(1500);

            Povit2.setTargetPosition(0);

            Povit2.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            Povit2.setPower(-0.7);

            sleep(1000);
        } else if (propPos == 4) {
            turnToHeadingWithImu(imu, 270, this);

            sleep(2000);

            turnToHeadingWithImu(imu, 180, this);

            sleep(2000);

            turnToHeadingWithImu(imu, 0, this);
        }
    }

    public void encoderDrive(double speed, double TargetCm) {

        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        int newLeftTarget;
        int newRightTarget;

        // Determine new target position, and pass to motor controller
        newLeftTarget = LF.getCurrentPosition() + (int)(TargetCm * COUNTS_PER_CM);
        newRightTarget = RF.getCurrentPosition() + (int)(TargetCm * COUNTS_PER_CM);

        LF.setTargetPosition(newLeftTarget);
        RF.setTargetPosition(newRightTarget);

        // Turn On RUN_TO_POSITION
        LF.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        RF.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        LF.setPower(speed);
        RF.setPower(speed);

        while (LF.isBusy() && RF.isBusy()) {
            //do nothing
        }

        // Stop all motion
        LF.setPower(0);
        RF.setPower(0);

        // Turn off RUN_TO_POSITION
        LF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RF.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void turnToHeadingWithImu(BNO055IMU imu, double targetHeading, LinearOpMode opMode){

        double p = 0.005;
        double d = 0.00001;

        PIDFController turnSpeedPID = new PIDFController(p, 0, d, 0);

        double ConvertedHeading;

        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (heading <= 0) {
            ConvertedHeading = (360 + heading);
            ConvertedHeading = (360-ConvertedHeading);
        } else {
            ConvertedHeading = (0 + heading);
            ConvertedHeading = (360-ConvertedHeading);
        }

        double headingError = targetHeading - ConvertedHeading;

        while (opMode.opModeIsActive() && (Math.abs(headingError) > 2)) {

            heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (heading <= 0) {
                ConvertedHeading = (360 + heading);
                ConvertedHeading = (360-ConvertedHeading);
            } else {
                ConvertedHeading = (0 + heading);
                ConvertedHeading = (360-ConvertedHeading);
            }

            headingError = targetHeading - ConvertedHeading;

            if (headingError < -180) {
                headingError = (360 + headingError);
            }else if (headingError > 180) {
                headingError = (headingError - 360);
            }

            double turnSpeed = turnSpeedPID.calculate(-headingError);

            LF.setPower(-turnSpeed);
            RF.setPower(turnSpeed);

            telemetry.addData("left", ConvertedHeading);
            telemetry.addData("right", turnSpeed);
            telemetry.addData("heading error", headingError);
            telemetry.update();

        }

        LF.setPower(0);
        RF.setPower(0);

    }

}
