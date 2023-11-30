package org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Odometry;

@TeleOp
public class TestMaxVelocity extends LinearOpMode {

    Odometry odo = new Odometry();
    Drivetrain drive = new Drivetrain();

    ElapsedTime elapsedTime = new ElapsedTime();

    double maxVecticalVelo;
    double maxHorzontalVelo;

    double maxVecticalAcc;
    double maxHorzontalAcc;

    double lastTime;

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        odo.init(hardwareMap);
        drive.init(hardwareMap);

        waitForStart();

        getAcc();

        while (opModeIsActive()){
            telemetry.addData("Distance", maxVecticalAcc);
            telemetry.addData("time", lastTime);
            telemetry.update();
        }

//        while (opModeIsActive()){
//
//            double vertical = -gamepad1.right_stick_y;
//            double horizontal = gamepad1.right_stick_x*1.5;
//            double pivot = gamepad1.left_stick_x;
//
//            drive.RF.setPower(-pivot + (vertical - horizontal));
//            drive.RB.setPower(-pivot + (vertical + horizontal));
//            drive.LF.setPower(pivot + (vertical + horizontal));
//            drive.LB.setPower(pivot + (vertical - horizontal));
//
//            double differenceX = odo.getMaxVerticalVelocity() - maxVecticalVelo;
//
//            if (differenceX > 0){
//                lastTime = elapsedTime.seconds();
//                maxVecticalVelo += differenceX;
//            }
//
//            double differenceY = odo.getMaxHorizontalVelocity() - maxHorzontalVelo;
//
//            if (differenceY > 0){
//                lastTime = elapsedTime.seconds();
//                maxHorzontalVelo += differenceY;
//            }
//
//            if (differenceX < 0){
//                double timeDifference = elapsedTime.seconds() - lastTime;
//                maxVecticalAcc = odo.X / timeDifference;
//            } else if (differenceY < 0) {
//                double timeDifference = elapsedTime.seconds() - lastTime;
//                maxHorzontalAcc = odo.Y / timeDifference;
//            }
//
////            double differenceAccX = accX - maxVecticalAcc;
////
////            if (differenceAccX > 0){
////                maxVecticalAcc += accX;
////            }
////
////            double differenceAccY = accY - maxHorzontalAcc;
////
////            if (differenceAccY > 0){
////                maxHorzontalAcc += accY;
////            }
//
//            telemetry.addData("x velo", differenceX);
//            telemetry.addData("y velo", differenceY);
//            telemetry.addData("x acc", maxVecticalAcc);
//            telemetry.addData("y acc", maxHorzontalAcc);
//            telemetry.update();
//
//        }
    }

    public void getAcc(){

        drive.RF.setPower(1);
        drive.LF.setPower(1);
        drive.RB.setPower(1);
        drive.LB.setPower(1);

        elapsedTime.reset();

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

        lastTime = elapsedTime.seconds();

    }
}
