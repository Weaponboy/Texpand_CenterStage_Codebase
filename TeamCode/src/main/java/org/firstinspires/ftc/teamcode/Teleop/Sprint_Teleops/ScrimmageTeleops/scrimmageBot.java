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
@Disabled
public class scrimmageBot extends OpMode {

    DcMotor RightDrive;

    DcMotor LeftDrive;

    DcMotor Pivot;

    Servo Gripper;

    Double Slow = 1.0;

    ElapsedTime runtime = new ElapsedTime();

    int buttondelaytime = 300;


    public Gamepad currentGamepad1;

    public Gamepad previousGamepad1;

    ColorSensor right_Pixel;
    float[] hsvval = new float[3];

    private static final double MIN_HUE = 0; // Minimum hue value for the target color
    private static final double MAX_HUE = 360; // Maximum hue value for the target color
    private static final double MIN_SATURATION = 0; // Minimum saturation value for the target color
    private static final double MAX_SATURATION = 100; // Maximum saturation value for the target color
    private static final double MIN_VALUE = 0; // Minimum value (brightness) for the target color
    private static final double MAX_VALUE = 100; // Maximum value (brightness) for the target color
    boolean isTargetColorDetected = false;

    @Override
    public void init() {

        RightDrive = hardwareMap.get(DcMotor.class,"RightDrive");
        LeftDrive = hardwareMap.get(DcMotor.class,"LeftDrive");
        Pivot = hardwareMap.get(DcMotor.class,"Pivot");
        Gripper = hardwareMap.get(Servo.class,"Gripper");

        RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        right_Pixel = hardwareMap.get(ColorSensor.class, "right_Pixel");

    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        double vertical = gamepad1.left_stick_y*1.2*Slow;
        double pivot = -gamepad1.right_stick_x*1.2*Slow;

        double denominator = Math.max(Math.abs(vertical) + Math.abs(pivot), 1);

//        if (currentGamepad1.a && !previousGamepad1.a && Gripper.getPosition() < 1) {
//            Gripper.setPosition(0.5);
//        } else if (currentGamepad1.a && !previousGamepad1.a && Gripper.getPosition() > 0.9) {
//            Gripper.setPosition(1);
//        }



        if (gamepad1.start){
            Gripper.setPosition(0.4);

        }

        if (gamepad1.back){
            Gripper.setPosition(1);

        }


        if (gamepad1.a && Slow == 1.0 && runtime.milliseconds() > buttondelaytime) {
            Slow = 0.3;
            runtime.reset();

        }

        else if (gamepad1.a && Slow == 0.3 && runtime.milliseconds() > buttondelaytime){
            Slow = 1.0;
            runtime.reset();
        }


        RightDrive.setPower(0.6*((vertical - pivot)/denominator));
        LeftDrive.setPower(0.6*((vertical + pivot)/denominator));

        telemetry.addData("PivotPosition", Pivot.getCurrentPosition());
        telemetry.update();

        if (gamepad1.x) {
            Pivot.setTargetPosition(0);
            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot.setPower(0.7);
        }

        if (gamepad1.b && Pivot.getCurrentPosition() < 1400) {
            Pivot.setTargetPosition(1400);
            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot.setPower(0.7);
        } else if (gamepad1.b && Pivot.getCurrentPosition() > 1400) {
            Pivot.setTargetPosition(1400);
            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot.setPower(-0.7);
        }

        if (gamepad1.right_trigger > 0 && Pivot.getCurrentPosition() > 1300){
            if(Pivot.getCurrentPosition() < 1480){
                Pivot.setTargetPosition(Pivot.getCurrentPosition()+5);
            }
            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            Pivot.setPower(0.4);
        }

        if (gamepad1.y) {
            Pivot.setTargetPosition(1020);
            Pivot.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            if (Pivot.getCurrentPosition()>971) {
                Pivot.setPower(-0.7);
            }
            if (Pivot.getCurrentPosition()<971) {
                Pivot.setPower(0.7);
            }
        }

    }
}

