package org.firstinspires.ftc.teamcode.hardware._;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.Objects;

public class  Delivery_Slides {

    HardwareMap hardwareMap;

    DcMotorEx Left_Slide;
    DcMotorEx Right_Slide;

    boolean SlideSafetyHeight = false;
    boolean SlideSafetyBottom = false;

    public enum SlideState{
        manual,
        moving,
        targetReached
    }

    SlideState slideState = SlideState.manual;

    public Delivery.GripperState updateSlides(Gamepad gamepad1, Gamepad gamepad2, Delivery.armState state){

        Delivery.GripperState targetGripperState = null;

        SlideSafetyHeight = Left_Slide.getCurrentPosition() > 2200;
        SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;

        switch (slideState){

            case manual:

                if (gamepad2.right_stick_y < -0.5 && !SlideSafetyHeight || gamepad1.x && !SlideSafetyHeight) {
                    SlideSafetyHeight = Left_Slide.getCurrentPosition() > 2200;
                    SlidesBothPower(0.4);
                    if (Objects.requireNonNull(state) == Delivery.armState.collect) {
                        targetGripperState = Delivery.GripperState.closed;
                    }
                } else if (gamepad2.right_stick_y > 0.5 && !SlideSafetyBottom || gamepad1.a && !SlideSafetyBottom) {
                    SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;
                    SlidesBothPower(-0.4);
                    if (Objects.requireNonNull(state) == Delivery.armState.collect) {
                        targetGripperState = Delivery.GripperState.closed;
                    }
                }else if (gamepad2.left_stick_y > 0.5 && !SlideSafetyBottom) {
                    SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;
                    SlidesBothPower(-0.6);
                    if (Objects.requireNonNull(state) == Delivery.armState.collect) {
                        targetGripperState = Delivery.GripperState.closed;
                    }
                } else {
                    if (getCurrentposition() < 100){
                        SlidesBothPower(0);
                    }else {
                        SlidesBothPower(0.0005);
                    }
                }

                break;
            case moving:

                targetGripperState = Delivery.GripperState.closed;

                if (Math.abs(Left_Slide.getVelocity()) < 2){
                    slideState = SlideState.targetReached;
                    Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                }

                break;
            case targetReached:
                slideState = SlideState.manual;
                break;
            default:

        }

        return targetGripperState;
    }

    public void init(HardwareMap Hmap){

        hardwareMap = Hmap;

        Left_Slide = hardwareMap.get(DcMotorEx.class, "Left_Slide");
        Right_Slide = hardwareMap.get(DcMotorEx.class, "Right_Slide");

        Right_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void DeliverySlides(int setpoint, double power){

        Right_Slide.setTargetPosition(setpoint);
        Left_Slide.setTargetPosition(setpoint);

        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Right_Slide.setPower(power);
        Left_Slide.setPower(power);

    }

    public double getCurrentDraw(){
        return (Left_Slide.getCurrent(CurrentUnit.MILLIAMPS) + Right_Slide.getCurrent(CurrentUnit.MILLIAMPS))/2;
    }

    public void SlidesBothPower(double power){
        Right_Slide.setPower(power);
        Left_Slide.setPower(power);
    }

    public int getCurrentposition(){
        return Right_Slide.getCurrentPosition() + Left_Slide.getCurrentPosition()/2;
    }

    public SlideState getSlideState() {
        return slideState;
    }

    public void setSlideState(SlideState slideState) {
        this.slideState = slideState;
    }

}
