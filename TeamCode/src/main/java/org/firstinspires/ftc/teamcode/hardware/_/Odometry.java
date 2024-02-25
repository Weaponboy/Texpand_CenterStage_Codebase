package org.firstinspires.ftc.teamcode.hardware._;

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

import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;

public class Odometry {

    DcMotorEx LF;
    DcMotorEx RF;
    DcMotorEx LB;
    DcMotorEx RB;

    public DcMotorEx leftPod;
    public DcMotorEx rightPod;
    public DcMotorEx centerPod;

    HardwareMap hardwareMap;

    public double trackwidth = 36.65835571289;
    public static double rightPodOffset = 18.949177856445;
    public double centerPodOffset = 18.7227783203125;
    public static double ticks_per_degree = 476;
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
        this.heading = startHeading;
        this.startHeading = startHeading;
    }

    public Odometry(double startX, double startY, double startHeading, boolean imu){
        this.X = startX;
        this.Y = startY;
        this.startHeading = startHeading;
    }

    public Odometry(){
        this.X = 0;
        this.Y = 0;
        this.heading = 0;
    }

    public double X, Y, heading;
    public double headingRaw;

    public double dtheta;

    public double dx;
    public double dy;

    public double factor = 0;

    public BNO055IMU imu = null;

    Orientation YawAngle;

    public double correctedStart = 0;

    double yI = 0;
    double xI = 0;

    ElapsedTime resetHeading = new ElapsedTime();

    public void update(){

        double lastHeading = heading;

        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        botHeading = -YawAngle.firstAngle;

        headingRaw = botHeading;

        botHeading += getCorrectStartHeading(startHeading);

        if (botHeading <= 0) {
            heading = (360 + botHeading);
        } else {
            heading = (0 + botHeading);
        }

        double headingTheta;

        if (lastHeading < 2 && heading > 358){
            double adaptedHeading = (360 - heading)+lastHeading;
            headingTheta = ticks_per_degree * (adaptedHeading);
        }else if (heading < 2 && lastHeading > 358){
            double adaptedHeading = (360 - lastHeading) + heading;
            headingTheta = ticks_per_degree * (adaptedHeading);
        } else {
            headingTheta = ticks_per_degree * (heading - lastHeading);
        }

        oldCenterPod = currentCenterPod;
        oldRightPod = currentRightPod;

        currentCenterPod = -centerPod.getCurrentPosition();
        currentRightPod = rightPod.getCurrentPosition();

        int dn2 = currentRightPod - oldRightPod;
        int dn3 = currentCenterPod - oldCenterPod;

        dx = cm_per_tick * (dn2 - (headingTheta) * rightPodOffset / trackwidth);
        dy = cm_per_tick * (dn3 - (headingTheta) * centerPodOffset / trackwidth);

        X += dx * Math.cos(Math.toRadians(heading)) - dy * Math.sin(Math.toRadians(heading));
        Y += dx * Math.sin(Math.toRadians(heading)) + dy * Math.cos(Math.toRadians(heading));

//        oldCenterPod = currentCenterPod;
//        oldLeftPod = currentLeftPod;
//        oldRightPod = currentRightPod;
//
//        currentCenterPod = -centerPod.getCurrentPosition();
//        currentLeftPod = -leftPod.getCurrentPosition();
//        currentRightPod = rightPod.getCurrentPosition();
//
//        int dn1 = currentLeftPod - oldLeftPod;
//        int dn2 = currentRightPod - oldRightPod;
//        int dn3 = currentCenterPod - oldCenterPod;
//
//        //(rpD-lpD)/trackwidth
//        dtheta = Math.toDegrees(cm_per_tick * ((dn1-dn2) / trackwidth));
//        dx = cm_per_tick * (dn1+dn2)/2.0;
//        dy = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);
//
//        double theta = heading + (dtheta / 2.0);
//        X += dx * Math.cos(Math.toRadians(theta)) - dy * Math.sin(Math.toRadians(theta));
//        Y += dx * Math.sin(Math.toRadians(theta)) + dy * Math.cos(Math.toRadians(theta));
//        heading += dtheta;
//
//        if (heading > 360) {
//            heading = heading - 360;
//        } else if (heading < 0){
//            heading = heading + 360;
//        }
//
//        if (resetHeading.milliseconds() > 1000){
//            reset(getIMUHeading());
//            resetHeading.reset();
//        }

    }

    public void updateIMU(){

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

    }

    public void updateArc(){

        oldCenterPod = currentCenterPod;
        oldLeftPod = currentLeftPod;
        oldRightPod = currentRightPod;

        currentCenterPod = -centerPod.getCurrentPosition();
        currentLeftPod = -leftPod.getCurrentPosition();
        currentRightPod = rightPod.getCurrentPosition();

        int dn1 = currentLeftPod - oldLeftPod;
        int dn2 = currentRightPod - oldRightPod;
        int dn3 = currentCenterPod - oldCenterPod;

        dtheta = Math.toDegrees(cm_per_tick * ((dn1-dn2) / trackwidth));

        if (dtheta == 0){
            dtheta = Math.toDegrees(cm_per_tick * ((dn1-dn2) / trackwidth));
            dx = cm_per_tick * (dn1+dn2)/2.0;
            dy = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);
        }else {
            double dfoward = cm_per_tick * (dn1+dn2)/2.0;
            double dstrafe = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);

            double xR = dfoward/dtheta;
            double yR = dstrafe/dtheta;

            dx = (xR*Math.sin(dtheta)) + (yR*Math.sin(dtheta));
            dy = (xR - xR * Math.cos(dtheta)) - (yR - yR * Math.cos(dtheta));
        }

        double theta = heading + (dtheta / 2.0);
        X += dx * Math.cos(Math.toRadians(theta)) - dy * Math.sin(Math.toRadians(theta));
        Y += dx * Math.sin(Math.toRadians(theta)) + dy * Math.cos(Math.toRadians(theta));
        heading += dtheta;

        if (heading > 360) {
            heading = heading - 360;
        } else if (heading < 0){
            heading = heading + 360;
        }

    }

    public void update(double delta){

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
//
//        heading = 0;
//
//        heading += getCorrectStartHeading(startHeading);
//
//        if (heading <= 0) {
//            ConvertedHeadingForPosition = (360 + heading);
//        } else {
//            ConvertedHeadingForPosition = (0 + heading);
//        }
//
//        heading = ConvertedHeadingForPosition;
//
//        oldCenterPod = currentCenterPod;
//        oldLeftPod = currentLeftPod;
//        oldRightPod = currentRightPod;
//
//        currentCenterPod -= delta;
//        currentLeftPod -= delta;
//        currentRightPod += delta;
//
//        int dn1 = currentLeftPod - oldLeftPod;
//        int dn2 = currentRightPod - oldRightPod;
//        int dn3 = currentCenterPod - oldCenterPod;
//
//        dx = cm_per_tick * (dn1+dn2)/2.0;
//        dy = cm_per_tick * (dn3 - (dn2-dn1) * centerPodOffset / trackwidth);
//
//        X += dx * Math.cos(Math.toRadians(heading)) - dy * Math.sin(Math.toRadians(heading));
//        Y += dx * Math.sin(Math.toRadians(heading)) + dy * Math.cos(Math.toRadians(heading));

    }

    public void init(HardwareMap hardwareMap2){

        resetHeading.reset();

        hardwareMap = hardwareMap2;

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

        drive = new Drivetrain();

        drivePID = new PIDFController(driveP, 0, driveD, driveF);

        strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);

        PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        leftPod = hardwareMap.get(DcMotorEx.class, "LF");
        centerPod = hardwareMap.get(DcMotorEx.class, "LB");
        rightPod = hardwareMap.get(DcMotorEx.class, "RF");

        rightPod.setDirection(DcMotorSimple.Direction.REVERSE);
        leftPod.setDirection(DcMotorSimple.Direction.REVERSE);
        centerPod.setDirection(DcMotorSimple.Direction.REVERSE);

        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        drive.init(hardwareMap);
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

    public double getIMUHeading(){
        YawAngle = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        botHeading = -YawAngle.firstAngle;

        botHeading += getCorrectStartHeading(startHeading);

        if (botHeading <= 0) {
            ConvertedHeadingForPosition = (360 + botHeading);
        } else {
            ConvertedHeadingForPosition = (0 + botHeading);
        }

        return ConvertedHeadingForPosition;
    }

    public static double getMaxVelocity(){
        return maxYVelocity;
    }

    public void Odo_Drive(double targetX, double targetY, double targetRot) {

        double driveP = 0.06;
        double driveD = 0.001;
        double driveF = 0;

        double strafeP = 0.04;
        double strafeD = 0.001;
        double strafeF = 0;

        double rotationP = 0.02;
        double rotationD = 0.001;
        double rotationF = 0;

        drivePID.setPIDF(driveP, 0, driveD, driveF);
        strafePID.setPIDF(strafeP, 0, strafeD, strafeF);
        PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

        boolean targetReached;

        do {

            update();

            Xdist = (targetX - X);
            Ydist = (targetY - Y);

            rotdist = (targetRot - ConvertedHeading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 360) {
                rotdist = (rotdist - 360);
            }

            if(Math.abs(getHorizontalVelocity()) < 3){
                yI += 0.005;
            }else {
                yI = 0;
            }

            if(Math.abs(getVerticalVelocity()) < 3){
                xI += 0.005;
            }else {
                xI = 0;
            }

            drivePID.setPIDF(driveP, xI, driveD, driveF);
            strafePID.setPIDF(strafeP, yI, strafeD, strafeF);
            PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

            RRXdist = Ydist * Math.sin(Math.toRadians(heading)) + Xdist * Math.cos(Math.toRadians(heading));
            RRYdist = Ydist * Math.cos(Math.toRadians(heading)) - Xdist * Math.sin(Math.toRadians(heading));

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

        }while ((Math.abs(Xdist) > 1.4 ) || (Math.abs(Ydist) > 1.4 ) || (Math.abs(rotdist) > 1.4));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public void Odo_Drive_New(double targetX, double targetY, double targetRot) {

        double driveP = 0.06;
        double driveD = 0.001;
        double driveF = 0;

        double strafeP = 0.04;
        double strafeD = 0.001;
        double strafeF = 0;

        double rotationP = 0.02;
        double rotationD = 0.001;
        double rotationF = 0;

        drivePID.setPIDF(driveP, 0, driveD, driveF);
        strafePID.setPIDF(strafeP, 0, strafeD, strafeF);
        PivotPID.setPIDF(rotationP, 0, rotationD, rotationF);

        boolean targetReached;

        do {

            update();


            drivePID.setPIDF(0.02, xI, 0.0001, 0);
            strafePID.setPIDF(0.03, yI, 0.0001, 0);

            Vector2D error;
            PathingPower correctivePower = new PathingPower();

            error = new Vector2D( targetX - X,  targetY - Y);

            double xDist = error.getX();
            double yDist = error.getY();

            double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
            double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

            double xPower = drivePID.calculate(-robotRelativeXError)*1.2;
            double yPower = strafePID.calculate(-robotRelativeYError)*1.3;

            double rotdist = (targetRot - heading);

            if (rotdist < -180) {
                rotdist = (360 + rotdist);
            } else if (rotdist > 360) {
                rotdist = (rotdist - 360);
            }

            PivotPID = new PIDController(rotationP, 0, rotationD);

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

        }while ((Math.abs(Xdist) > 1.4 ) || (Math.abs(Ydist) > 1.4 ) || (Math.abs(rotdist) > 1.4));

        drive.RF.setPower(0);
        drive.RB.setPower(0);
        drive.LF.setPower(0);
        drive.LB.setPower(0);

    }

    public boolean Odo_Drive_Teleop(double targetX, double targetY, double targetRot) {

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

        update();

        Xdist = (targetX - X);
        Ydist = (targetY - Y);

        rotdist = (targetRot - ConvertedHeading);

        if (rotdist < -180) {
            rotdist = (360 + rotdist);
        } else if (rotdist > 360) {
            rotdist = (rotdist - 360);
        }

        RRXdist = Ydist * Math.sin(Math.toRadians(heading)) + Xdist * Math.cos(Math.toRadians(heading));
        RRYdist = Ydist * Math.cos(Math.toRadians(heading)) - Xdist * Math.sin(Math.toRadians(heading));

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

        return Math.abs(Xdist) > 1.2 || Math.abs(Ydist) > 1.2 || Math.abs(rotdist) > 1.2;

    }

    public double getCorrectStartHeading(double globalStartHeading){

        int StartHeading = (int) Math.round(globalStartHeading);

        if (StartHeading == 270){
            correctedStart = -90;
        }else if (StartHeading == 180) {
            correctedStart = -180;
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

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public void reset(double heading){

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        this.heading = heading;

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }

    public void reset(Vector2D newPos, double heading){

        leftPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerPod.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        X = newPos.getX();
        Y = newPos.getY();
        this.heading = heading;

        leftPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerPod.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


    }
}
