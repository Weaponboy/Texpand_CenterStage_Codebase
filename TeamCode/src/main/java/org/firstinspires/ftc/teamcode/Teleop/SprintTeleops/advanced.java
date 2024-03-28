package org.firstinspires.ftc.teamcode.Teleop.SprintTeleops;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp
public class advanced extends OpMode {

    DcMotor RightDrive;

    DcMotor LeftDrive;

    DcMotor Slide;
    DcMotor SlidePivot;

    ServoImplEx LeftGripper;
    ServoImplEx RightGripper;

    Double Slow = 1.0;

    ElapsedTime runtime = new ElapsedTime();

    int buttondelaytime = 300;
    double PivotServoposition;

    public Gamepad currentGamepad1;

    public Gamepad previousGamepad1;

    int MaxSlide = -2050;
    int MaxPivotSlide = -25;


    @Override
    public void init() {

        RightDrive = hardwareMap.get(DcMotor.class,"RightDrive");
        LeftDrive = hardwareMap.get(DcMotor.class,"LeftDrive");
        SlidePivot = hardwareMap.get(DcMotor.class,"SlidePivot");
        Slide = hardwareMap.get(DcMotor.class,"Slide");
        RightGripper = hardwareMap.get(ServoImplEx.class,"RightGripper");
        LeftGripper = hardwareMap.get(ServoImplEx.class,"LeftGripper");


        RightGripper.setPwmRange(new PwmControl.PwmRange(600, 2500));
        LeftGripper.setPwmRange(new PwmControl.PwmRange(600, 2500));

        RightGripper.setDirection(Servo.Direction.FORWARD);
        LeftGripper.setDirection(Servo.Direction.REVERSE);

        RightDrive.setDirection(DcMotorSimple.Direction.REVERSE);

        RightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        LeftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        SlidePivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        SlidePivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


        Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        currentGamepad1 = new Gamepad();
        previousGamepad1 = new Gamepad();
        RightGripper.setPosition(0);
        LeftGripper.setPosition(0);
    }

    @Override
    public void loop() {

        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        double vertical = gamepad1.left_stick_y;
        double pivot = -gamepad1.left_stick_x*1.2*0.2;


        double denominator = Math.max(Math.abs(vertical) + Math.abs(pivot), 1);

        if (gamepad1.start){
            LeftGripper.setPosition(0);
            RightGripper.setPosition(0);
        }

        if (gamepad1.back){
            LeftGripper.setPosition(0.3);
            RightGripper.setPosition(0.3);
        }


        RightDrive.setPower(((vertical - pivot)/denominator));
        LeftDrive.setPower(((vertical + pivot)/denominator));

        telemetry.addData("Slide Position", Slide.getCurrentPosition());
        telemetry.addData("Slide Pivot Position", SlidePivot.getCurrentPosition());
        telemetry.addData("PivotPosition", SlidePivot.getCurrentPosition());
        telemetry.update();

        if (gamepad1.x && SlidePivot.getCurrentPosition() < -1) {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SlidePivot.setPower(0.08);
        }else if (gamepad1.right_trigger > 0 && SlidePivot.getCurrentPosition() > -70) {
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            SlidePivot.setPower(-gamepad1.right_trigger);
        }else{
            SlidePivot.setPower(0);
        }


        if(gamepad1.dpad_down && Slide.getCurrentPosition() < -10){
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setPower(0.5);
        } else if (gamepad1.dpad_up && Slide.getCurrentPosition() > MaxSlide){
            Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            Slide.setPower(-0.5);
        }else{
            Slide.setPower(0);
        }
//
//        if( Pivot.getCurrentPosition() < 800) {
//            PivotServoposition = 0.3;
//            PivotServo.setPosition(PivotServoposition);
//            telemetry.addData("PivotServoposition 800", Pivot.getCurrentPosition());
//        }else if( Pivot.getCurrentPosition() > 1200) {
//            PivotServoposition = 0.3;
//            PivotServo.setPosition(PivotServoposition);
//            telemetry.addData("PivotServoposition 1200", Pivot.getCurrentPosition());
//        }else if( Pivot.getCurrentPosition() > 1000) {
//            PivotServoposition = 0.5;
//            PivotServo.setPosition(PivotServoposition);
//            telemetry.addData("PivotServoposition 1000", Pivot.getCurrentPosition());
//        }else if( Pivot.getCurrentPosition() > 900) {
//            PivotServoposition = 0.4;
//            PivotServo.setPosition(PivotServoposition);
//            telemetry.addData("PivotServoposition 900", Pivot.getCurrentPosition());
//        }
//
//        telemetry.addData("Climbposition", Climb.getCurrentPosition());
//        telemetry.update();
//
//        if (gamepad1.right_bumper){
//            Climb.setTargetPosition(maxClimbheight);
//            Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Climb.setPower(-0.8);
//        }
//
//        if (gamepad1.left_bumper) {
//            Climb.setTargetPosition(0);
//            Climb.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//            Climb.setPower(0.5);
//        }
//
//        if (gamepad1.left_stick_button && gamepad1.right_stick_button){
//            planelauncher.setPosition(0.5);
//        }
    }
}


