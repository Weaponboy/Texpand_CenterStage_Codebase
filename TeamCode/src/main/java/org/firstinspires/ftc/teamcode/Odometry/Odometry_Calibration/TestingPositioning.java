package org.firstinspires.ftc.teamcode.Odometry.Odometry_Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@TeleOp
public class TestingPositioning extends OpMode {

    Odometry odometry = new Odometry(210, 337, 90);

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    @Override
    public void init() {
        odometry.init(hardwareMap);
    }

    @Override
    public void loop() {
        odometry.update();
        dashboardTelemetry.addData("x", odometry.X);
        dashboardTelemetry.addData("y", odometry.Y);
        dashboardTelemetry.addData("heading", odometry.heading);
        dashboardTelemetry.update();
    }

}
