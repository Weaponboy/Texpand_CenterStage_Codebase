package org.firstinspires.ftc.teamcode.Odometry.Odometry_Calibration;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

@TeleOp
public class Testing_Pid_Controller extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    PIDController headingPID;
    double ki = 0;

    @Override
    public void init() {
        headingPID =  new PIDController(rotationP, ki, rotationD);
    }

    @Override
    public void loop() {

        ki += 1;

        headingPID.setPID(rotationP, ki, rotationD);

        dashboardTelemetry.addData("x", headingPID.calculate(10));
        dashboardTelemetry.update();

        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
    }

}
