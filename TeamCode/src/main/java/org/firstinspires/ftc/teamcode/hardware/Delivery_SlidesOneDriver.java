package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import java.util.ArrayList;

public class Delivery_SlidesOneDriver {

    HardwareMap hardwareMap;

    public DcMotorEx Left_Slide;
    public DcMotorEx Right_Slide;

    ArrayList<Double> slidesProfile = new ArrayList<>();

    boolean SlideSafetyHeight = false;
    boolean SlideSafetyBottom = false;

    int startPosition;

    double maxVelo = 2784;
    double maxAccel = maxVelo/1.8;

    public int getTargetPos() {
        return targetPos;
    }

    public void setTargetPos(int targetPos) {
        this.targetPos = targetPos;
    }

    int targetPos;

    public enum SlideState{
        manual,
        moving,
        targetReached
    }

    SlideState slideState = SlideState.manual;

    public Delivery.GripperState updateSlides(Gamepad gamepad1, Gamepad gamepad2, Delivery.armState state){

        Delivery.GripperState targetGripperState = null;

        SlideSafetyHeight = Left_Slide.getCurrentPosition() > 3000;
        SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;

        switch (slideState){

            case manual:

                if (gamepad1.left_stick_y < -0.7 && !SlideSafetyHeight) {
                    SlideSafetyHeight = Left_Slide.getCurrentPosition() > 3000;
                    SlidesBothPower(0.6);
                } else if (gamepad1.left_stick_y > 0.7 && !SlideSafetyBottom) {
                    SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;
                    SlidesBothPower(-0.6);
                }else {
                    if (getCurrentposition() > 20 && getCurrentposition() < 50){
                        Right_Slide.setPower(-0.5);
                        Left_Slide.setPower(-0.5);
                    }else if (getCurrentposition() < 20){
                        SlidesBothPower(0);
                    }else {
                        SlidesBothPower(0.0005);
                    }
                }

                break;
            case moving:

                targetGripperState = Delivery.GripperState.closed;

                if (Left_Slide.getTargetPosition() == 0){

                    if (Math.abs(getVelocity()) < 3 && Math.abs(Left_Slide.getCurrentPosition()- Left_Slide.getTargetPosition()) < 50){

                        slideState = SlideState.targetReached;

                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    }

                }else {
                    if (Math.abs(getVelocity()) < 3 && Math.abs(Left_Slide.getCurrentPosition()- Left_Slide.getTargetPosition()) < 50){

                        slideState = SlideState.targetReached;

                        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

                    }
                }

                break;
            case targetReached:
                slideState = SlideState.manual;
                break;
            default:

        }

        return targetGripperState;
    }

    public void updateSlides(Gamepad gamepad1, Gamepad gamepad2){

        Delivery.GripperState targetGripperState = null;

        SlideSafetyHeight = Left_Slide.getCurrentPosition() > 2200;
        SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;

        switch (slideState){

            case manual:

                if (gamepad1.left_stick_y < -0.7 && !SlideSafetyHeight || gamepad1.x && !SlideSafetyHeight) {
                    SlideSafetyHeight = Left_Slide.getCurrentPosition() > 2200;
                    setTargetPos(getCurrentposition() + 40);
                    motionProfile();
                } else if (gamepad1.left_stick_y > 0.7 && !SlideSafetyBottom || gamepad1.a && !SlideSafetyBottom) {
                    SlideSafetyBottom = Left_Slide.getCurrentPosition() < 15;
                    setTargetPos(getCurrentposition() - 40);
                    motionProfile();
                }

                SlidesBothPower(getPower());

                break;
            case moving:

                targetGripperState = Delivery.GripperState.closed;

                if (Math.abs(Left_Slide.getVelocity()) < 2 && Math.abs(getCurrentposition()- Left_Slide.getTargetPosition()) < 50){

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

    }

    public void motionProfile(){

        slidesProfile.clear();

        int decelerationNumber = 0;

        startPosition = getCurrentposition();

        int targetPosition = targetPos;

        int deceleration_dt;

        if (targetPosition > startPosition){
            deceleration_dt = (int) ((maxVelo * maxVelo) / (maxAccel * 2));
        }else{
            deceleration_dt = (int) ((maxVelo * maxVelo) / ((maxVelo/6) * 2));
        }

        int halfway_distance = Math.abs(targetPosition - startPosition) / 2;

        if (deceleration_dt > halfway_distance){
            deceleration_dt = halfway_distance;
        }

        System.out.println("deceleration_dt" + deceleration_dt);

        System.out.println("startPosition" + startPosition);

        System.out.println("targetPosition" + targetPosition);

        int range;

        for (int i = 0; i < Math.abs(targetPosition - startPosition) - 1; i++) {

            if (i + deceleration_dt >= Math.abs(targetPosition - startPosition)){

                decelerationNumber += 1;

                range = Math.abs(deceleration_dt - decelerationNumber);

                double DecSlope = (double) range / (double) Math.abs(deceleration_dt) * 100;

                DecSlope = DecSlope*0.01;

                int currentPoint;
                int nextPoint;
                double delta;

                if (startPosition > targetPosition){
                    currentPoint = startPosition - i;
                    nextPoint = startPosition - (i + 1);
                    delta = nextPoint - currentPoint;
                }else {
                    currentPoint = startPosition + i;
                    nextPoint = startPosition + (i + 1);
                    delta = nextPoint - currentPoint;
                }

                System.out.println("delta" + delta);

                double power = delta * DecSlope;

                System.out.println("slide Power" + power);

                slidesProfile.add(power);

            }else {

                int currentPoint;
                int nextPoint;

                double delta;

                if (startPosition > targetPosition){
                    currentPoint = startPosition - i;
                    nextPoint = startPosition - (i + 1);
                    delta = nextPoint - currentPoint;
                }else {
                    currentPoint = startPosition + i;
                    nextPoint = startPosition + (i + 1);
                    delta = nextPoint - currentPoint;
                }

                System.out.println("delta" + delta);

                System.out.println("slide Power" + delta);

                slidesProfile.add((double) delta);
            }

        }

    }

    public double getPower(){

        int index;

        if (startPosition > targetPos){
            index = startPosition - getCurrentposition();
        } else {
            index = getCurrentposition() - startPosition;
        }

        if (index+1 >= slidesProfile.size()){
            if (getCurrentposition() < 10){
                return 0;
            }else {
                return 0.0005;
            }
        }else {
            return slidesProfile.get(index);
        }

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

        Right_Slide.setTargetPosition(setpoint-40);
        Left_Slide.setTargetPosition(setpoint-40);

        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        Right_Slide.setPower(power);
        Left_Slide.setPower(power);

    }

    public void runToPosition(int setpoint){

        Right_Slide.setTargetPosition(setpoint);
        Left_Slide.setTargetPosition(setpoint);

        Right_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Left_Slide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if(setpoint > getCurrentposition()){
            Right_Slide.setPower(1);
            Left_Slide.setPower(1);
        }else {
            Right_Slide.setPower(-1);
            Left_Slide.setPower(-1);
        }


    }

    public double getCurrentDraw(){
        return (Left_Slide.getCurrent(CurrentUnit.MILLIAMPS) + Right_Slide.getCurrent(CurrentUnit.MILLIAMPS))/2;
    }

    public double getVelocity(){
        return (Left_Slide.getVelocity() + Right_Slide.getVelocity())/2;
    }

    public void SlidesBothPower(double power){
        Right_Slide.setPower(power);
        Left_Slide.setPower(power);
    }

    public void resetZero(){
        Right_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Right_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Left_Slide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public int getCurrentposition(){
        return (Right_Slide.getCurrentPosition() + Left_Slide.getCurrentPosition())/2;
    }

    public SlideState getSlideState() {
        return slideState;
    }

    public void setSlideState(SlideState slideState) {
        this.slideState = slideState;
    }

}
