package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

public class Collection {

    DcMotorEx Intake;

    public Servo IntakeHeightLeft;
    public Servo IntakeHeightRight;

    ServoImplEx sweeper;

    HardwareMap hmap;

    double collect = 0.234;
    double startingBox = 0.95;
    double stowed = 0.25;
    double letClawThrough = 0.25;

    double firstPixel = 0.234;
    double firstAndHalf = 0.265;
    double secondPixel = 0.304;
    double secondAndHalf = 0.365;
    double thirdPixel = 0.384;
    double thirdAndHalf = 0.416;
    double forthPixel = 0.4444;
    double forthAndHalf = 0.456;
    double fifthPixel = 0.479;
    double fifthAndHalf = 0.512;

    intakePowerState statePower = intakePowerState.off;
    intakeHeightState heightState = intakeHeightState.startingBox;
    sweeperState sweeperstate = sweeperState.retract;

    public enum sweeperState{
        push,
        retract
    }

    public enum intakePowerState{
        on,
        off,
        reversed,
        reversedHalf,
        onHalf
    }

    public enum intakeHeightState{
        startingBox,
        collect,
        stowed,
        letClawThrough,
        firstPixel,
        firstAndHalf,
        secondPixel,
        secondAndHalf,
        thirdPixel,
        thirdAndHalf,
        forthPixel,
        forthAndHalf,
        fifthPixel,
        fifthAndHalf
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
            case reversedHalf:
                Intake.setPower(-0.6);
                break;
            case onHalf:
                Intake.setPower(0.55);
                break;
            default:
        }

    }

    public void updateSweeper(){

        switch (sweeperstate){
            case push:
                sweeper.setPosition(0);
                break;
            case retract:
                sweeper.setPosition(0.13);
                break;
            default:
        }

    }

    public void updateIntakeHeight(){

        switch (heightState){
            case startingBox:
                IntakeHeightRight.setPosition(startingBox);
//                IntakeHeightLeft.setPosition(hangStowed);
                break;
            case stowed:
                IntakeHeightRight.setPosition(stowed);
//                IntakeHeightLeft.setPosition(stowed);
                break;
            case collect:
                IntakeHeightRight.setPosition(collect);
//                IntakeHeightLeft.setPosition(collect);
                break;
            case letClawThrough:
                IntakeHeightRight.setPosition(letClawThrough);
//                IntakeHeightLeft.setPosition(letClawThrough);
                break;
            case firstPixel:
                IntakeHeightRight.setPosition(firstPixel);
//                IntakeHeightLeft.setPosition(firstPixel);
                break;
            case secondPixel:
                IntakeHeightRight.setPosition(secondPixel);
//                IntakeHeightLeft.setPosition(secondPixel);
                break;
            case thirdPixel:
                IntakeHeightRight.setPosition(thirdPixel);
//                IntakeHeightLeft.setPosition(thirdPixel);
                break;
            case forthPixel:
                IntakeHeightRight.setPosition(forthPixel);
//                IntakeHeightLeft.setPosition(forthPixel);
                break;
            case fifthPixel:
                IntakeHeightRight.setPosition(fifthPixel);
//                IntakeHeightLeft.setPosition(fifthPixel);
                break;
            case firstAndHalf:
                IntakeHeightRight.setPosition(firstAndHalf);
//                IntakeHeightLeft.setPosition(firstPixel);
                break;
            case secondAndHalf:
                IntakeHeightRight.setPosition(secondAndHalf);
//                IntakeHeightLeft.setPosition(secondPixel);
                break;
            case thirdAndHalf:
                IntakeHeightRight.setPosition(thirdAndHalf);
//                IntakeHeightLeft.setPosition(thirdPixel);
                break;
            case forthAndHalf:
                IntakeHeightRight.setPosition(forthAndHalf);
//                IntakeHeightLeft.setPosition(forthPixel);
                break;
            case fifthAndHalf:
                IntakeHeightRight.setPosition(fifthAndHalf);
//                IntakeHeightLeft.setPosition(fifthPixel);
                break;
            default:
        }

    }

    public void init(HardwareMap hardwareMap){

        hmap = hardwareMap;

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

//        sweeper = hardwareMap.get(ServoImplEx.class, "sweeper");
//
//        sweeper.setPwmRange(new PwmControl.PwmRange(600, 2400));
//
//        sweeper.setPosition(0.13);

        IntakeHeightRight = hardwareMap.get(Servo.class, "IntakeServoRight");

        IntakeHeightRight.setDirection(Servo.Direction.FORWARD);

//        IntakeHeightRight.setPosition(startingBox);
//
//        IntakeHeightLeft = hardwareMap.get(Servo.class, "IntakeServoLeft");
//
//        IntakeHeightLeft.setDirection(Servo.Direction.FORWARD);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public void setSweeperState(sweeperState heightState) {
        this.sweeperstate = heightState;
    }

    public double getIntakePower() {
        return Intake.getPower();
    }

    public double getIntakeHeightRight() {
        return IntakeHeightRight.getPosition();
    }

    public double getIntakeHeight() {
        return (IntakeHeightRight.getPosition() + IntakeHeightLeft.getPosition())/2;
    }

    public double getIntakeCurrentUse(){
        return Intake.getCurrent(CurrentUnit.MILLIAMPS);
    }
}
