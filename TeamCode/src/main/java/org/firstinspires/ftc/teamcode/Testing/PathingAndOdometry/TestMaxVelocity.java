package org.firstinspires.ftc.teamcode.Testing.PathingAndOdometry;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@TeleOp
//@Disabled
public class TestMaxVelocity extends LinearOpMode {

    Odometry odo = new Odometry();
    Drivetrain drive = new Drivetrain();

    ElapsedTime elapsedTimeX = new ElapsedTime();
    ElapsedTime elapsedTimeY = new ElapsedTime();

    double maxVecticalVelo;
    double maxHorzontalVelo;

    double maxVecticalAcc;
    double maxHorzontalAcc;

    double lastTimeX;
    double lastTimeY;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odo.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();

//        getAcc();

//        while (opModeIsActive()){
//            telemetry.addData("Distance", maxVecticalAcc);
//            telemetry.addData("time", lastTime);
//            telemetry.update();
//        }

        while (opModeIsActive()){

            double vertical = -gamepad1.right_stick_y;
            double horizontal = gamepad1.right_stick_x*1.5;
            double pivot = gamepad1.left_stick_x;

            drive.RF.setPower(-pivot + (vertical - horizontal));
            drive.RB.setPower(-pivot + (vertical + horizontal));
            drive.LF.setPower(pivot + (vertical + horizontal));
            drive.LB.setPower(pivot + (vertical - horizontal));

            double differenceX = odo.getVerticalVelocity()- maxVecticalVelo;

            if (differenceX > 0){
                maxVecticalVelo = odo.getVerticalVelocity();
            }

            if (odo.getVerticalVelocity() < 5 && maxVecticalVelo < 190){
                elapsedTimeX.reset();
            }

            if (maxVecticalVelo > 190 && lastTimeX < 1){
                lastTimeX = elapsedTimeX.seconds();
            }

            double differenceY = odo.getHorizontalVelocity() - maxHorzontalVelo;

            if (differenceY > 0){
                maxHorzontalVelo = odo.getHorizontalVelocity();
            }

            if (odo.getHorizontalVelocity() < 5 && maxHorzontalVelo < 190){
                elapsedTimeY.reset();
            }

            if (maxHorzontalVelo > 130 && lastTimeY < 1){
                lastTimeY = elapsedTimeY.seconds();
            }

            telemetry.addData("x velo", maxVecticalVelo);
            telemetry.addData("y velo", maxHorzontalVelo);
            telemetry.addData("lastTime", lastTimeX);
            telemetry.addData("lastTime", lastTimeY);
            telemetry.update();

        }
    }

    public void getAcc(){

        drive.RF.setPower(1);
        drive.LF.setPower(1);
        drive.RB.setPower(1);
        drive.LB.setPower(1);

        elapsedTimeX.reset();

        while (odo.X < 80){
            odo.update();

            drive.RF.setPower(1);
            drive.LF.setPower(-1);
            drive.RB.setPower(1);
            drive.LB.setPower(-1);

            maxVecticalAcc = odo.getVerticalVelocity();
        }

        drive.RF.setPower(0);
        drive.LF.setPower(0);
        drive.RB.setPower(0);
        drive.LB.setPower(0);

        maxVecticalAcc = odo.getVerticalVelocity();

        lastTimeX = elapsedTimeX.seconds();

    }

}
