package org.firstinspires.ftc.teamcode.hardware.SubSystems;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.botHeading;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveF;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.maxYVelocity;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationF;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeF;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.drive;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;

public class Odometry {

    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx LB;
    DcMotorEx RB;

    DcMotorEx leftPod;
    DcMotorEx rightPod;
    DcMotorEx centerPod;

    HardwareMap hardwareMap;

    public double trackwidth = 36;
    public double centerPodOffset = 17;
    public double wheelRadius = 1.75;
    public double podTicks = 8192;

    public double cm_per_tick = 2.0 * Math.PI * wheelRadius / podTicks;

    public int currentRightPod = 0;
    public int currentLeftPod = 0;
    public int currentCenterPod = 0;

    public int oldRightPod = 0;
    public int oldLeftPod = 0;
    public int oldCenterPod = 0;

    public double startX, startY, startHeading;

    public PIDFController drivePID;
    public PIDFController strafePID;
    public PIDFController PivotPID;

    public double Xdist = 0;
    public double Ydist = 0;

    public double rotdist = 0;

    public double RRXdist = 0;
    public double RRYdist = 0;

    public double Horizontal = 0;
    public double Vertical = 0;

    public double Pivot = 0;

    public static double ConvertedHeading = 0;

    public static double ConvertedHeadingForPosition = 0;

    public Odometry(double startX, double startY, double startHeading){
        this.X = startX;
        this.Y = startY;
        this.startHeading = startHeading;
        this.heading = startHeading;
    }

    public Odometry(){
        this.X = 0;
        this.Y = 0;
        this.startHeading = 0;
    }

    public double X, Y, heading;

    public double dtheta;

    public double dx;
    public double dy;

    public double factor = 0;

    public BNO055IMU imu = null;

    Orientation YawAngle;

    public double correctedStart = 0;

    public void update(){

        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        botHeading = -YawAngle.firstAngle;

        botHeading += getCorrectStartHeading(startHeading);

        if (botHeading <= 0) {
            ConvertedHeadingForPosition = (360 + botHeading);
        } else {
            ConvertedHeadingForPosition = (0 + botHeading);
        }

        heading = ConvertedHeadingForPosition;

        oldCenterPod = currentCenterPod;
        oldLeftPod = currentLeftPod;
        oldRightPod = currentRightPod;

        currentCenterPod = -centerPod.getCurrentPosition();
        currentLeftPod = -leftPod.getCurrentPosition();
        currentRightPod = rightPod.getCurrentPosition();

        int dn1 = currentLeftPod - oldLeftPod;
        int dn2 = currentRightPod - oldRightPod;
        int dn3 = currentCenterPod - oldCenterPod;

        dtheta = cm_per_tick * ((dn2-dn1) / trackwidth);
        dx = cm_per_tick * (dn1+dn2)/2.0;
        dy = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);

        double theta = heading + (dtheta / 2.0);
        X += dx * Math.cos(Math.toRadians(ConvertedHeadingForPosition)) - dy * Math.sin(Math.toRadians(ConvertedHeadingForPosition));
        Y += dx * Math.sin(Math.toRadians(ConvertedHeadingForPosition)) + dy * Math.cos(Math.toRadians(ConvertedHeadingForPosition));
        heading += dtheta;

        factor = heading/360;

        if(factor > 1) {
            heading = heading - 360*(int)factor;
        }

    }

    public void update(double delta){

        heading = 0;

        heading += getCorrectStartHeading(startHeading);

        if (heading <= 0) {
            ConvertedHeadingForPosition = (360 + heading);
        } else {
            ConvertedHeadingForPosition = (0 + heading);
        }

        heading = ConvertedHeadingForPosition;

        oldCenterPod = currentCenterPod;
        oldLeftPod = currentLeftPod;
        oldRightPod = currentRightPod;

        currentCenterPod -= delta;
        currentLeftPod -= delta;
        currentRightPod += delta;

        int dn1 = currentLeftPod - oldLeftPod;
        int dn2 = currentRightPod - oldRightPod;
        int dn3 = currentCenterPod - oldCenterPod;

        dx = cm_per_tick * (dn1+dn2)/2.0;
        dy = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);

        X += dx * Math.cos(Math.toRadians(heading)) - dy * Math.sin(Math.toRadians(heading));
        Y += dx * Math.sin(Math.toRadians(heading)) + dy * Math.cos(Math.toRadians(heading));

    }

    public void init(HardwareMap hardwareMap2){

       hardwareMap = hardwareMap2;

        LF = hardwareMap.get(DcMotorEx.class, "LF");
        LB = hardwareMap.get(DcMotorEx.class, "LB");
        RF = hardwareMap.get(DcMotorEx.class, "RF");
        RB = hardwareMap.get(DcMotorEx.class, "RB");

        RF.setDirection(DcMotorSimple.Direction.FORWARD);
        LF.setDirection(DcMotorSimple.Direction.REVERSE);
        RB.setDirection(DcMotorSimple.Direction.FORWARD);
        LB.setDirection(DcMotorSimple.Direction.REVERSE);

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        drive = new Drivetrain();

        drive.init(hardwareMap);

        drivePID = new PIDFController(driveP, 0, driveD, driveF);

        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);

        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        RF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LF.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LB.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        RF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        RB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LF.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        LB.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftPod = LF;
        rightPod = RF;
        centerPod = LB;
    }

    public void resetHeadingUsingImu(){

        update();

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        botHeading = -YawAngle.firstAngle;

        heading = Math.toRadians(botHeading);

    }

    public void updateIMUHeading(){
        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        botHeading = -YawAngle.firstAngle;
    }

    public static double getMaxVelocity(){
        return maxYVelocity;
    }

    public void Odo_Drive(double targetX, double targetY, double targetRot) {

        double driveP = 0.1;
        double driveD = 0.007;
        double driveF = 0;

        double strafeP = 0.1;
        double strafeD = 0.005;
        double strafeF = 0;

        double rotationP = 0.04;
        double rotationD = 0.001;
        double rotationF = 0;

        drivePID.setPIDF(driveP, 0, driveD, driveF);
        strafePID.setPIDF(strafeP, 0, strafeD, strafeF);
        PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

        do {

            update();

            Xdist = (targetX - X);
            Ydist = (targetY - Y);

            if (botHeading <= 0) {
                ConvertedHeading = (360 + botHeading);
            } else {
                ConvertedHeading = (0 + botHeading);
            }

            rotdist = (targetRot - ConvertedHeading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 360) {
                rotdist = (rotdist - 360);
            }

            RRXdist = Ydist * Math.sin(Math.toRadians(ConvertedHeading)) + Xdist * Math.cos(Math.toRadians(ConvertedHeading));
            RRYdist = Ydist * Math.cos(Math.toRadians(ConvertedHeading)) - Xdist * Math.sin(Math.toRadians(ConvertedHeading));

            Vertical = drivePID.calculate(-RRXdist);
            Horizontal = strafePID.calculate(-RRYdist);
            Pivot = PivotPID.calculate(-rotdist);

            double denominator = Math.max(Math.abs(Vertical) + Math.abs(Horizontal) + Math.abs(Pivot), 1);

            double left_Front = (Vertical + Horizontal + Pivot) / denominator;
            double left_Back = (Vertical - Horizontal + Pivot) / denominator;
            double right_Front = (Vertical - Horizontal - Pivot) / denominator;
            double right_Back = (Vertical + Horizontal - Pivot) / denominator;

            drive.RF.setPower(right_Front);
            drive.RB.setPower(right_Back);
            drive.LF.setPower(left_Front);
            drive.LB.setPower(left_Back);

        }while ((Math.abs(Xdist) > 1.2 ) || (Math.abs(Ydist) > 1.2 ) || (Math.abs(rotdist) > 1.2));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public double getCorrectStartHeading(double globalStartHeading){

        int StartHeading = (int) Math.round(globalStartHeading);

        if (StartHeading == 270){
            correctedStart = -90;
        } else if (StartHeading == 90) {
            correctedStart = 90;
        }
        return correctedStart;
    }

    public double getVerticalVelocity(){
        double tickVelo = leftPod.getVelocity() + rightPod.getVelocity()/2;
        double cmVelo = tickVelo*cm_per_tick;
        return cmVelo;
    }

    public double getHorizontalVelocity(){
        double tickVelo = centerPod.getVelocity();
        double cmVelo = tickVelo*cm_per_tick;
        return cmVelo;
    }

    public void reset(Vector2D newPos){

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        X = newPos.getX();
        Y = newPos.getY();

    }
}
