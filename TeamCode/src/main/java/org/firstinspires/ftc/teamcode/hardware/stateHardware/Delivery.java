package org.firstinspires.ftc.teamcode.hardware.stateHardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree.Sprint_3_teleop;


public class Delivery {

    DcMotorEx Pivot;

    ServoImplEx secondPivotLeft;
    ServoImplEx secondPivotRight;

    ServoImplEx RightClaw;
    ServoImplEx LeftClaw;

    ServoImplEx RotateClaw;

    Servo mainPivotLeft;
    Servo mainPivotRight;

    ServoImplEx secondRotate;

    ElapsedTime pivotMoveTime = new ElapsedTime();

    double timeToWait;

    double timePerDegree = 7;

    double collectTopPivotPos = 0.1;
    double deliveryTopPivot = 1;
    double safeTopPivot = 0.3;

    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.97;
    double deliverySecondPivot = 0.3;

    double clawOpen = 0.25;
    double clawClosed = 0;

    double rotateCollect = 0.5;
    double rotateRight = 1;

    HardwareMap hmap;

    enum armState{
        delivery,
        collect,
        moving,
    }

    enum gripperState{
        bothOpen,
        bothClosed,
        leftOpen,
        rightOpen
    }

    armState armstateTarget = armState.collect;
    armState armstateCurrent = armState.collect;

    gripperState gripperstate = gripperState.bothClosed;

    public void updateArm (){

        switch (armstateTarget){

            case delivery:

                switch (armstateCurrent){
                    case moving:
                        break;
                    case collect:
                        armstateCurrent = armState.moving;

                        pivotMoveTime.reset();

                        timeToWait = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - collectTopPivotPos) * 180) * timePerDegree);

                        setClaws(clawClosed);

                        setSecondPivot(collectSecondPivot);

                        setMainPivot(collectTopPivotPos);

                        RotateClaw.setPosition(rotateCollect);

                        break;
                    case delivery:
                        break;
                    default:
                }

                break;
            case collect:

                switch (armstateCurrent){
                    case moving:
                        break;
                    case collect:
                        break;
                    case delivery:

                        armstateCurrent = armState.moving;

                        pivotMoveTime.reset();

                        timeToWait = Math.max((Math.abs(getSecondPivotPosition()-deliverySecondPivot)*180)*timePerDegree, (Math.abs(getTopPivotPosition()-deliveryTopPivot)*180)*timePerDegree);

                        setClaws(clawClosed);

                        setSecondPivot(deliverySecondPivot);

                        setMainPivot(deliveryTopPivot);

                        RotateClaw.setPosition(rotateCollect);

                        break;
                    default:
                }

                break;
            default:
        }

        switch (armstateCurrent){
            case moving:
                switch (armstateTarget){
                    case delivery:
                        if (pivotMoveTime.milliseconds() >= timeToWait){
                            armstateCurrent = armState.delivery;
                        }
                        break;
                    case collect:
                        if (pivotMoveTime.milliseconds() >= timeToWait){
                            armstateCurrent = armState.collect;
                        }
                        break;
                    default:
                }
                break;
            default:
        }
    }

    public void init(HardwareMap hardwareMap){
        hmap = hardwareMap;

        Pivot = hardwareMap.get(DcMotorEx.class, "pivot");

        Pivot.setDirection(DcMotorSimple.Direction.REVERSE);

        Pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Pivot.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        pivotMoveTime.reset();

        /**servos init*/

        secondPivotLeft = hardwareMap.get(ServoImplEx.class, "RightPivot");
        secondPivotRight = hardwareMap.get(ServoImplEx.class, "LeftPivot");

        secondPivotRight.setDirection(Servo.Direction.REVERSE);

        secondPivotLeft.setPwmRange(new PwmControl.PwmRange(600, 2500));
        secondPivotRight.setPwmRange(new PwmControl.PwmRange(600, 2500));

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

    public double getSecondPivotPosition(){
        return secondPivotLeft.getPosition() + secondPivotRight.getPosition()/2;
    }

    public double getMainPivotPosition(){
        return mainPivotLeft.getPosition() + mainPivotRight.getPosition()/2;
    }

    private void setMainPivot(double position){
        mainPivotLeft.setPosition(position);
        mainPivotRight.setPosition(position);
    }

    public double getTopPivotPosition(){
        return secondPivotLeft.getPosition();
    }

    public void setClaws(double position){
        LeftClaw.setPosition(position);
        RightClaw.setPosition(position);
    }

    private void setSecondPivot(double position){
        secondPivotLeft.setPosition(position);
        secondPivotRight.setPosition(position);
    }

    public void setArmTargetState(armState targetState) {
        this.armstateTarget = targetState;
    }

    public armState getArmState() {
        return armstateCurrent;
    }

}
