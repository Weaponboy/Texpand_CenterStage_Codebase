package org.firstinspires.ftc.teamcode.hardware.Odometry.Odometry_Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Odometry;

@Config
@TeleOp
public class get_Odometry_Encoder_Values extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();


    Odometry odo = new Odometry(0, 0, 0);

    @Override
    public void init() {
        odo.init(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        odo.update();
        telemetry.addData("left pod", odo.currentLeftPod);
        telemetry.addData("right pod", odo.currentRightPod);
        telemetry.addData("center pod", odo.currentCenterPod);
        telemetry.update();
    }

}
