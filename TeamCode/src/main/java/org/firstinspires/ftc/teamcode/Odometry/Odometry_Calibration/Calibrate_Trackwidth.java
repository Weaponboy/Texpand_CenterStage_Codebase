package org.firstinspires.ftc.teamcode.Odometry.Odometry_Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

@Config
@TeleOp
public class Calibrate_Trackwidth extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    public static double trackwidth = 35.96864350492449;
    public static double centerPodOffset = 17.57500474911514;
    public double wheelRadius = 1.75;
    public double podTicks = 8192;

    public double cm_per_tick = 2.0 * Math.PI * wheelRadius / podTicks;

    public int currentRightPod = -85437;
    public int currentLeftPod = 84105;
    public int currentCenterPod = -78319;

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

    public double XdistForStop = 0;
    public double YdistForStop = 0;

    public double rotdistForStop = 0;

    public double RRXdist = 0;
    public double RRYdist = 0;
    public double Horizontal = 0;
    public double Vertical = 0;

    public double Pivot = 0;

    public double ConvertedHeading = 0;

    public double X = startX, Y = startY, heading = startHeading;

    public double dtheta;

    public double dx;
    public double dy;

    public double factor = 0;

    public BNO055IMU imu = null;

    Orientation YawAngle;

    @Override
    public void init() {}

    @Override
    public void loop() {

        dx = cm_per_tick * (currentRightPod+currentLeftPod)/2.0;
        dy = cm_per_tick * (currentCenterPod - (currentRightPod-currentLeftPod) * centerPodOffset / trackwidth);

        X = dx * Math.cos(0) - dy * Math.sin(0);
        Y = dx * Math.sin(0) + dy * Math.cos(0);

        dashboardTelemetry.addData("track", trackwidth);
        dashboardTelemetry.addData("centerPodOffset", centerPodOffset);
        dashboardTelemetry.addData("X", dx);
        dashboardTelemetry.addData("Y", dy);
        dashboardTelemetry.update();

//        packet.put("trackwidth", trackwidth);
//        packet.put("centerPodOffset", centerPodOffset);
//        packet.put("X", X);
//        packet.put("Y", Y);
//
//        dashboard.sendTelemetryPacket(packet);
        
    }

}
