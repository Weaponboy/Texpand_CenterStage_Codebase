package org.firstinspires.ftc.teamcode.Testing.PathingAndOdometry;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@TeleOp
public class TestingTurning extends LinearOpMode {

    Odometry odometry = new Odometry(0, 0, 0);

    Drivetrain drive = new Drivetrain();

    mecanumFollower follower = new mecanumFollower();

    LynxModule controlHUb;

    boolean goodBattery;

    @Override
    public void runOpMode() throws InterruptedException {
        odometry.init(hardwareMap);
        drive.init(hardwareMap);
        controlHUb = (LynxModule) hardwareMap.get(LynxModule.class, "Control Hub");

        if (controlHUb.getInputVoltage(VoltageUnit.VOLTS)>13){
            goodBattery = true;
        }
        waitForStart();

        while (opModeIsActive()){
            odometry.update();

            double pivotPower;

            if (goodBattery){
                pivotPower = follower.getTurnPower(90, odometry.heading, 0.02, 0.0008);
            } else {
                pivotPower = follower.getTurnPower(90, odometry.heading, 0.03, 0.001);
            }

            vertical = 0;
            horizontal = 0;

            drive.RF.setPower((-pivotPower + (vertical - horizontal)));
            drive.RB.setPower((-pivotPower + (vertical + horizontal)));
            drive.LF.setPower((pivotPower + (vertical + horizontal)));
            drive.LB.setPower((pivotPower + (vertical - horizontal)));

            telemetry.addData("goodBattery", goodBattery);
            telemetry.addData("x", odometry.X);
            telemetry.addData("y", odometry.Y);
            telemetry.addData("heading", odometry.heading);
            telemetry.update();
        }


    }

}
