package org.firstinspires.ftc.teamcode.Auto;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous
public class Scrimmage_Bot_Auto extends LinearOpMode {

    DcMotor LF;
    DcMotor RF;
    DcMotor Povit2;
    Servo PivotLeft;

    static final double     COUNTS_PER_MOTOR_REV    = 960;
    static final double     DRIVE_GEAR_REDUCTION    = 1;
    static final double     WHEEL_DIAMETER_INCM   = 9;
    static final double     COUNTS_PER_CM        = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCM * 3.1415);

    public BNO055IMU imu = null;

    @Override
    public void runOpMode() throws InterruptedException {

        RF = hardwareMap.get(DcMotor.class,"RF");
        LF = hardwareMap.get(DcMotor.class,"LF");
        Povit2 = hardwareMap.get(DcMotor.class, "Pivot2");
        PivotLeft = hardwareMap.get(Servo.class, "PivotLeft");

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

        PivotLeft.setPosition(1);

        waitForStart();

        //start code

        encoderDrive(0.4, -35);

        turnToHeadingWithImu(imu, -100, this);

        sleep(3);

        encoderDrive(0.4, -70);

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

        double p = 0.05;
        double d = 0;

        PIDFController turnSpeedPID = new PIDFController(p, 0, d, 0);

        double headingError = targetHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        while (opMode.opModeIsActive() && (Math.abs(headingError) > 0.8)) {

            headingError = targetHeading - imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

            if (headingError > 180){
                headingError -= 360;
            }else if (headingError <= -180){
                headingError += 360;
            }

            // Determine required steering to keep on heading
//            double turnSpeed = Range.clip(headingError * 0.009, -0.2, 0.2);

            double turnSpeed = turnSpeedPID.calculate(headingError);

            LF.setPower(-turnSpeed);
            RF.setPower(turnSpeed);

            telemetry.addData("heading error", headingError);
            telemetry.update();

        }

        LF.setPower(0);
        RF.setPower(0);

    }
    
}
