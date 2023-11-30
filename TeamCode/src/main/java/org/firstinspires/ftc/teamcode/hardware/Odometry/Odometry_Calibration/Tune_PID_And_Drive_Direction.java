package org.firstinspires.ftc.teamcode.hardware.Odometry.Odometry_Calibration;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.Horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.Pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.Vertical;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveF;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.driveP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationF;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeF;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.strafeP;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Odometry;

@Disabled
@Config
public class Tune_PID_And_Drive_Direction extends OpMode {

    Vector2D robotPos = new Vector2D();

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Odometry odo = new Odometry(0,0,0);

    public double heading = 90;
    double Xdist;
    double Ydist;
    double rotdist;

    @Override
    public void init() {
        robotPos.set(0,0);
        odo.drivePID = new PIDFController(driveP, 0, driveD, driveF);
        odo.strafePID = new PIDFController(strafeP, 0, strafeD, strafeF);
        odo.PivotPID = new PIDFController(rotationP, 0, rotationD, rotationF);

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {

        Odo_Drive(10, 10, 0);
    }

    public void Odo_Drive(double targetX, double targetY, double targetRot) {

        do {

            double CurrentXPos = robotPos.getX();

            double CurrentYPos = robotPos.getY();

            double Xdist = (targetX - CurrentXPos);
            double Ydist = (targetY - CurrentYPos);

            rotdist = (targetRot - heading);

            //CONVERT TARGET TO ROBOT RELATIVE TARGET
            double RRXdist = Ydist * Math.sin(Math.toRadians(heading)) + Xdist * Math.cos(Math.toRadians(heading));
            double RRYdist = Ydist * Math.cos(Math.toRadians(heading)) - Xdist * Math.sin(Math.toRadians(heading));

            //SET DRIVE CONSTANTS TO THE PIDF CONTROL LOOPS
            Vertical = odo.drivePID.calculate(-RRXdist);
            Horizontal = odo.strafePID.calculate(-RRYdist);
            Pivot = odo.PivotPID.calculate(-rotdist);

            double denominator = Math.max(Math.abs(Vertical) + Math.abs(Horizontal) + Math.abs(Pivot), 1);

            double left_Front = (Vertical + Horizontal + Pivot) / denominator;
            double left_Back = (Vertical - Horizontal + Pivot) / denominator;
            double right_Front = (Vertical - Horizontal - Pivot) / denominator;
            double right_Back = (Vertical + Horizontal - Pivot) / denominator;

//            telemetry.addData("rot ", rotdist);
//            telemetry.addData("pivot power", Pivot);
//            telemetry.addData("denominator", denominator);
//            telemetry.addData("right front", right_Front);
//            telemetry.addData("left front", left_Front);
//            telemetry.addData("right back", right_Back);
//            telemetry.addData("left back", left_Back);
//            telemetry.update();

        }while ((Math.abs(Xdist) > 0.8 ) || (Math.abs(Ydist) > 0.8 ) || (Math.abs(rotdist) > 0.8));

    }
}
