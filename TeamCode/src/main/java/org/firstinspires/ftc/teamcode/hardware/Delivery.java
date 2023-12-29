package org.firstinspires.ftc.teamcode.hardware;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;


public class Delivery {

    public DcMotorEx Pivot;

    public PIDFController pivot_controllers;

    public ServoImplEx pivot1;
    public ServoImplEx pivot2;

    public ServoImplEx RightClaw;
    public ServoImplEx LeftClaw;

    public ServoImplEx RotateClaw;

    public Servo mainPivotLeft;
    public Servo mainPivotRight;

    public ServoImplEx secondRotate;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    HardwareMap hmap;

    public void init(HardwareMap hardwareMap){
        hmap = hardwareMap;

        pivot_controllers = new PIDFController(pivot_p, pivot_i, pivot_d, 0.05);

        pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0.05);

        Pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        Pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        /**servos init*/

        pivot1 = hardwareMap.get(ServoImplEx.class, "RightPivot");
        pivot2 = hardwareMap.get(ServoImplEx.class, "LeftPivot");

        pivot2.setDirection(Servo.Direction.REVERSE);

        pivot1.setPwmRange(new PwmControl.PwmRange(600, 2500));
        pivot2.setPwmRange(new PwmControl.PwmRange(600, 2500));

        RightClaw = hardwareMap.get(ServoImplEx.class, "RightClaw");
        LeftClaw = hardwareMap.get(ServoImplEx.class, "LeftClaw");

        RightClaw.setDirection(Servo.Direction.FORWARD);
        LeftClaw.setDirection(Servo.Direction.REVERSE);

        RightClaw.setPwmRange(new PwmControl.PwmRange(600, 2000));
        LeftClaw.setPwmRange(new PwmControl.PwmRange(1000, 2400));

        mainPivotLeft = hardwareMap.get(ServoImplEx.class, "leftmain");
        mainPivotRight = hardwareMap.get(ServoImplEx.class, "rightmain");

        mainPivotLeft.setDirection(Servo.Direction.REVERSE);

        RotateClaw = hardwareMap.get(ServoImplEx.class, "ClawRotate");

        RotateClaw.setDirection(Servo.Direction.FORWARD);

        RotateClaw.setPwmRange(new PwmControl.PwmRange(700, 2400));

        secondRotate = hardwareMap.get(ServoImplEx.class, "secondRotate");

        secondRotate.setPwmRange(new PwmControl.PwmRange(750, 2400));

        secondRotate.setPosition(0.5);

    }

    public void setSecondPivot(double position){
        pivot1.setPosition(position);
        pivot2.setPosition(position);
    }

    public void setMainPivot(double position){
        mainPivotLeft.setPosition(position);
        mainPivotRight.setPosition(position);
    }

    public void setClaws(double position){
        LeftClaw.setPosition(position);
        RightClaw.setPosition(position);
    }

    public double getTopPivotPosition(){
        return pivot1.getPosition();
    }

    public double getSecondPivotPosition(){
        return mainPivotLeft.getPosition();
    }

}
