package org.firstinspires.ftc.teamcode.Odometry.Odometry_Calibration;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

@TeleOp
public class TestingTurning extends LinearOpMode {

    Odometry odometry = new Odometry(0, 0, 0);

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    TelemetryPacket packet = new TelemetryPacket();

    @Override
    public void runOpMode() throws InterruptedException {
        odometry.init(hardwareMap);
 
        waitForStart();

        odometry.Odo_Drive_Teleop(0, 0, 90);



        while (opModeIsActive()){
            odometry.update();

            telemetry.addData("x", odometry.X);
            telemetry.addData("y", odometry.Y);
            telemetry.addData("heading", odometry.heading);
            telemetry.update();
        }


    }

}
