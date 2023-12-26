package org.firstinspires.ftc.teamcode.hardware.stateHardware;

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
    double collect;
    double stowed;
    double letClawThrough;
    double firstPixel;
    double secondPixel;
    double thirdPixel;
    double forthPixel;
    double fifthPixel;

    intakePowerState statePower = intakePowerState.off;
    intakeHeightState heightState = intakeHeightState.stowed;

    private enum intakePowerState{
        on,
        off,
        reversed
    }

    private enum intakeHeightState{
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

    public void updateHeightState(){

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

    public void setHeightState(intakeHeightState heightState) {
        this.heightState = heightState;
    }

    public double getIntake() {
        return Intake.getPower();
    }

    public double getIntakeHeight() {
        return IntakeHeight.getPosition();
    }
}
