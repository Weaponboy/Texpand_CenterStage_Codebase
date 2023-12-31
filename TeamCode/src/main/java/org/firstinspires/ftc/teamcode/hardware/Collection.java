package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collection {

    DcMotorEx Intake;

    Servo IntakeHeight;

    HardwareMap hmap;

    //get the correct values when Ethan pushes
    double collect = 0;
    double stowed = 0.55;
    double letClawThrough = 0.2;
    double firstPixel = 0;
    double secondPixel = 0.175;
    double thirdPixel = 0.20;
    double forthPixel = 0.25;
    double fifthPixel = 0.275;

    intakePowerState statePower = intakePowerState.off;
    intakeHeightState heightState = intakeHeightState.stowed;

    public enum intakePowerState{
        on,
        off,
        reversed
    }

    public enum intakeHeightState{
        collect,
        stowed,
        letClawThrough,
        firstPixel,
        secondPixel,
        thirdPixel,
        forthPixel,
        fifthPixel
    }

    public void updateIntakeState(){

        switch (statePower){
            case on:
                Intake.setPower(1);
                break;
            case off:
                Intake.setPower(0);
                break;
            case reversed:
                Intake.setPower(-1);
                break;
            default:
        }

    }

    public void updateIntakeHeight(){

        switch (heightState){
            case stowed:
                IntakeHeight.setPosition(stowed);
                break;
            case collect:
                IntakeHeight.setPosition(collect);
                break;
            case letClawThrough:
                IntakeHeight.setPosition(letClawThrough);
                break;
            case firstPixel:
                IntakeHeight.setPosition(firstPixel);
                break;
            case secondPixel:
                IntakeHeight.setPosition(secondPixel);
                break;
            case thirdPixel:
                IntakeHeight.setPosition(thirdPixel);
                break;
            case forthPixel:
                IntakeHeight.setPosition(forthPixel);
                break;
            case fifthPixel:
                IntakeHeight.setPosition(fifthPixel);
                break;
            default:
        }

    }

    public void init(HardwareMap hardwareMap){

        hmap = hardwareMap;

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        IntakeHeight = hardwareMap.get(Servo.class, "IntakeServo");

        IntakeHeight.setDirection(Servo.Direction.FORWARD);

        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }

    public intakePowerState getPowerState() {
        return statePower;
    }

    public void setState(intakePowerState statePower) {
        this.statePower = statePower;
    }

    public intakeHeightState getHeightState() {
        return heightState;
    }

    public void setIntakeHeight(intakeHeightState heightState) {
        this.heightState = heightState;
    }

    public double getIntakePower() {
        return Intake.getPower();
    }

    public double getIntakeHeight() {
        return IntakeHeight.getPosition();
    }
}
