package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.ScrimmageTeleops;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class ServoBot extends OpMode {

    Servo GripperLeft;
    Servo GripperRight;
    Servo LeftDrive;
    Servo RightDrive;

    ElapsedTime runtime = new ElapsedTime();

    int buttondelaytime = 300;


    public Gamepad currentGamepad1;

    public Gamepad previousGamepad1;


    @Override
    public void init() {

        RightDrive = hardwareMap.get(Servo.class,"RightDrive");
        LeftDrive = hardwareMap.get(Servo.class,"LeftDrive");
        GripperLeft = hardwareMap.get(Servo.class,"GripperLeft");
        GripperRight = hardwareMap.get(Servo.class,"GripperRight");

        GripperRight.setDirection(Servo.Direction.REVERSE);
        RightDrive.setDirection(Servo.Direction.REVERSE);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();

    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        double vertical = (gamepad1.left_stick_y/2) + 0.5;
        double pivot = (-gamepad1.right_stick_x/2) + 0.5;

        boolean forward = gamepad1.left_stick_y > 0.5;
        boolean backwards = gamepad1.left_stick_y < -0.5;
        boolean turnLeft = gamepad1.left_stick_x > 0.5;
        boolean turnRight = gamepad1.left_stick_x < -0.5;

        if (forward){
            RightDrive.setPosition(1);
            LeftDrive.setPosition(1);
        } else if (backwards) {
            RightDrive.setPosition(0);
            LeftDrive.setPosition(0);
        }else if (turnRight) {
            RightDrive.setPosition(0);
            LeftDrive.setPosition(1);
        }else if (turnLeft) {
            RightDrive.setPosition(1);
            LeftDrive.setPosition(0);
        }else {
            RightDrive.setPosition(0);
            LeftDrive.setPosition(0.5);
        }

//        double denominator = Math.max(Math.abs(vertical) + Math.abs(pivot), 1);
//
//        RightDrive.setPosition(((vertical - pivot)/denominator));
//        LeftDrive.setPosition(((vertical + pivot)/denominator));

        telemetry.addData("Left Drive", LeftDrive.getPosition());
        telemetry.addData("Right Drive", RightDrive.getPosition());
        telemetry.update();

        if (gamepad1.start){
            SetGrippers(0.5);
        }

        if (gamepad1.back){
            SetGrippers(1);
        }

    }

    private void SetGrippers(double Position){
        GripperRight.setPosition(Position);
        GripperLeft.setPosition(Position);
    }
}


