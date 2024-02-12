package org.firstinspires.ftc.teamcode.hardware._;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Collection {

    DcMotorEx Intake;

    public Servo IntakeHeightLeft;
    public Servo IntakeHeightRight;

    HardwareMap hmap;

    double collect = 0;
    double hangStowed = 0.4;
    double stowed = 0.3;
    double letClawThrough = 0.25;
    double firstPixel = 0;
    double secondPixel = 0.14;
    double thirdPixel = 0.18;
    double forthPixel = 0.27;
    double fifthPixel = 0.31;

    intakePowerState statePower = intakePowerState.off;
    intakeHeightState heightState = intakeHeightState.stowed;

    public enum intakePowerState{
        on,
        off,
        reversed
    }

    public enum intakeHeightState{
        hangStowed,
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
            case hangStowed:
                IntakeHeightRight.setPosition(hangStowed);
                IntakeHeightLeft.setPosition(hangStowed);
                break;
            case stowed:
                IntakeHeightRight.setPosition(stowed);
                IntakeHeightLeft.setPosition(stowed);
                break;
            case collect:
                IntakeHeightRight.setPosition(collect);
                IntakeHeightLeft.setPosition(collect);
                break;
            case letClawThrough:
                IntakeHeightRight.setPosition(letClawThrough);
                IntakeHeightLeft.setPosition(letClawThrough);
                break;
            case firstPixel:
                IntakeHeightRight.setPosition(firstPixel);
                IntakeHeightLeft.setPosition(firstPixel);
                break;
            case secondPixel:
                IntakeHeightRight.setPosition(secondPixel);
                IntakeHeightLeft.setPosition(secondPixel);
                break;
            case thirdPixel:
                IntakeHeightRight.setPosition(thirdPixel);
                IntakeHeightLeft.setPosition(thirdPixel);
                break;
            case forthPixel:
                IntakeHeightRight.setPosition(forthPixel);
                IntakeHeightLeft.setPosition(forthPixel);
                break;
            case fifthPixel:
                IntakeHeightRight.setPosition(fifthPixel);
                IntakeHeightLeft.setPosition(fifthPixel);
                break;
            default:
        }

    }

    public void init(HardwareMap hardwareMap){

        hmap = hardwareMap;

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        IntakeHeightRight = hardwareMap.get(Servo.class, "IntakeServoRight");

        IntakeHeightRight.setDirection(Servo.Direction.REVERSE);

        IntakeHeightLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");

        IntakeHeightLeft.setDirection(Servo.Direction.FORWARD);

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
        return (IntakeHeightLeft.getPosition() + IntakeHeightLeft.getPosition())/2;
    }

    public double getIntakeCurrentUse(){
        return Intake.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
