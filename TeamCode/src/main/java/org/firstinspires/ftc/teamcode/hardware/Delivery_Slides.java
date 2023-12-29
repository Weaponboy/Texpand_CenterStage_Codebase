package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.slide_d;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.slide_i;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.slide_p;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree.Sprint_3_teleop;

public class  Delivery_Slides {

    HardwareMap hardwareMap;

    DcMotorEx Left_Slide;
    DcMotorEx Right_Slide;

    boolean SlideSafetyHeight = false;
    boolean SlideSafetyBottom = false;

    enum SlideState{
        manual,
        moving,
        targetReached;
    }

    SlideState slideState = SlideState.manual;

    public void updateSlides(Gamepad gamepad1, Gamepad gamepad2){

        SlideSafetyHeight = Left_Slide.getCurrentPosition() > 2200;
        SlideSafetyBottom = Left_Slide.getCurrentPosition() < 5;

        switch (slideState){

            case manual:
                if (gamepad1.x && !SlideSafetyHeight) {
                    SlideSafetyHeight = Left_Slide.getCurrentPosition() > 2200;
                    SlidesBothPower(0.3);
                } else if (gamepad1.a && !SlideSafetyBottom) {
                    SlideSafetyBottom = Left_Slide.getCurrentPosition() < 5;
                    SlidesBothPower(-0.3);
                }else {
                    SlidesBothPower(0.0005);
                }
                break;
            case moving:
                if (Left_Slide.getCurrentPosition() > (Left_Slide.getTargetPosition()-10) && Left_Slide.getCurrentPosition() < (Left_Slide.getTargetPosition()+10) ){
                    slideState = SlideState.targetReached;
                }
                break;
            case targetReached:
                slideState = SlideState.manual;
                break;
            default:

        }
    }

    public void init(HardwareMap Hmap){

        hardwareMap = Hmap;

        Left_Slide = hardwareMap.get(DcMotorEx.class, "Left_Slide");
        Right_Slide = hardwareMap.get(DcMotorEx.class, "Right_Slide");

        Left_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

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
