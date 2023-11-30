package org.firstinspires.ftc.teamcode.hardware.SubSystems;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.slide_d;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.slide_i;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.slide_p;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class  Delivery_Slides {

    HardwareMap hardwareMap;

    public PIDFController Slide_Power;

    public DcMotorEx Left_Slide;
    public DcMotorEx Right_Slide;

    public void init(HardwareMap Hmap){

        hardwareMap = Hmap;

        Left_Slide = hardwareMap.get(DcMotorEx.class, "Left_Slide");
        Right_Slide = hardwareMap.get(DcMotorEx.class, "Right_Slide");

        Left_Slide.setDirection(DcMotorSimple.Direction.REVERSE);

        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Slide_Power = new PIDFController(slide_p, slide_i, slide_d, 0);
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

}
