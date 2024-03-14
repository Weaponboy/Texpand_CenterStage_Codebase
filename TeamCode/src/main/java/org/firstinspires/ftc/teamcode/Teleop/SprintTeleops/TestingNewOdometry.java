package org.firstinspires.ftc.teamcode.Teleop.SprintTeleops;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

import java.util.Objects;

@TeleOp
public class TestingNewOdometry extends OpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(0,0,180);

    Collection collect = new Collection();

    @Override
    public void init() {
        
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        collect.init(hardwareMap);

    }

    @Override
    public void loop() {

        odometry.update();

        if (gamepad1.a && collect.IntakeHeightRight.getPosition() > 0.1){
            collect.IntakeHeightRight.setPosition(0);
        }else if (gamepad1.a && collect.IntakeHeightRight.getPosition() < 0.1){
            collect.IntakeHeightRight.setPosition(0.4);
        }

        vertical = -gamepad1.right_stick_y;
        horizontal = gamepad1.right_stick_x;
        pivot = gamepad1.left_stick_x;

        double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

        drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
        drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
        drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
        drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        telemetry.addData("centerPod", -odometry.centerPod.getCurrentPosition());
        telemetry.addData("rightPod", -odometry.rightPod.getCurrentPosition());
        telemetry.addData("X", odometry.X);
        telemetry.addData("Y", odometry.Y);
        telemetry.addData("heading", odometry.heading);
        telemetry.update();

    }
}
