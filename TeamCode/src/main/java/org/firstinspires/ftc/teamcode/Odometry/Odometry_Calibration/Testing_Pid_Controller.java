package org.firstinspires.ftc.teamcode.Odometry.Odometry_Calibration;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationD;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.rotationP;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathingUtility.PathingPower;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

@TeleOp
@Config
@Disabled
public class Testing_Pid_Controller extends OpMode {

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    PIDController XCorrective;
    PIDController YCorrective;

    Vector2D robotPositionVector = new Vector2D();
    Vector2D targetPosition = new Vector2D();

    mecanumFollower follower = new mecanumFollower();

    public static double verticalVelo = 0;
    public static double horizontalVelo = 0;

    public static double heading = 0;

    public static double xi = 0;
    public static double yi = 0;

    public static double x = 0;
    public static double y = 0;

    public static double xTarget = 0;
    public static double yTarget = 0;

    @Override
    public void init() {
        XCorrective =  new PIDController(rotationP, xi, rotationD);
        YCorrective =  new PIDController(rotationP, yi, rotationD);
    }

    @Override
    public void loop() {

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("xi", xi);
        packet.put("yi", yi);
        packet.put("heading", heading);
        packet.put("verticalVelo", verticalVelo);
        packet.put("horizontalVelo", horizontalVelo);
        packet.put("xTarget", xTarget);
        packet.put("xTarget", yTarget);
        packet.put("x", x);
        packet.put("y", y);
        packet.put("vertical", vertical);
        packet.put("horizontal", horizontal);
        dashboard.sendTelemetryPacket(packet);

        robotPositionVector.set(x, y);
        targetPosition.set(xTarget, yTarget);

        PathingPower correctivePower;
        PathingPower pathingPower;

        if(Math.abs(verticalVelo) < 2){
            xi += 0.00001;
        }else {
            xi = 0;
        }

        if(Math.abs(horizontalVelo) < 2){
            yi += 0.00001;
        }else {
            yi = 0;
        }

        pathingPower = new PathingPower(0,0);
        correctivePower = getCorrectivePowerAtEnd(robotPositionVector, targetPosition, heading);

        vertical = correctivePower.getVertical() + pathingPower.getVertical();
        horizontal = correctivePower.getHorizontal() + pathingPower.getHorizontal();

    }

    public PathingPower getCorrectivePowerAtEnd(Vector2D robotPos, Vector2D targetPos, double heading){

        XCorrective.setPID(0.03, xi, 0.002);
        YCorrective.setPID(0.04, yi, 0.001);

        Vector2D error;
        PathingPower correctivePower = new PathingPower();

        error = new Vector2D( targetPos.getX() - robotPos.getX(),  targetPos.getY() - robotPos.getY());

        double xDist = error.getX();
        double yDist = error.getY();

        double robotRelativeXError = yDist * Math.sin(Math.toRadians(heading)) + xDist * Math.cos(Math.toRadians(heading));
        double robotRelativeYError = yDist * Math.cos(Math.toRadians(heading)) - xDist * Math.sin(Math.toRadians(heading));

        double xPower = XCorrective.calculate(-robotRelativeXError)*1.2;
        double yPower = YCorrective.calculate(-robotRelativeYError)*1.3;

        correctivePower.set(xPower, yPower);

        return correctivePower;
    }

}
