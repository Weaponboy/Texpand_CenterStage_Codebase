package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class Delivery {

    ServoImplEx secondPivotLeft;
    ServoImplEx secondPivotRight;

    ServoImplEx RightClaw;
    ServoImplEx LeftClaw;

    ServoImplEx RotateClaw;

    Servo mainPivotLeft;
    Servo mainPivotRight;

    ServoImplEx secondRotate;

    ElapsedTime pivotMoveTimeDelivery = new ElapsedTime();
    ElapsedTime pivotMoveTimeCollection = new ElapsedTime();

    double timeToWaitDelivery;
    double timeToWaitCollection;

    double timePerDegree = 7;

    double collectTopPivotPos = 0.2;
    double deliveryTopPivot = 0.7;
    double safeTopPivot = 0.3;

    double deliveryTopPivotAuto = 1;
    double deliverySecondPivotAuto = 0.45;

    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.95;
    double deliverySecondPivot = -0.1;
    double lowdeliveryTopPivot = 1;

    double clawOpen = 0.35;
    double clawClosed = 0;

    double rotateCollect = 0.5;
    double rotateRight = 1;

    double secondRotateMiddle = 0.5;

    double targetRotatePos;
    static final double adjustFactor = 1.1;
    boolean closeToCollection;

    public void setMainPivotOffSet(double mainPivotOffSet) {
        this.mainPivotOffSet = mainPivotOffSet;
    }

    double mainPivotOffSet = 0;
    double targetMainPivot = 0;
    static final double servoPosPerTick = 0.00004100;
    static final double mainToSecondConst = 0.5/0.3;

    boolean DeliveryMovingAuto = false;
    boolean DeliveryMoving = false;
    boolean CollectionMoving = false;

    HardwareMap hmap;

    public enum armState{
        delivery,
        deliverAuto,
        collect,
        moving
    }

    public enum targetGripperState {
        openBoth,
        closeBoth,
        openLeft,
        openRight,
        closeLeft,
        closeRight
    }

    public enum leftGripperState{
        closed,
        open;
    }

    public enum rightGripperState{
        closed,
        open
    }

    armState armstateTarget = armState.moving;
    armState armstateCurrent = armState.collect;

    targetGripperState targetgripperState = targetGripperState.closeBoth;

    public rightGripperState getRightgripperstate() {
        return rightgripperstate;
    }

    public leftGripperState getLeftgripperstate() {
        return leftgripperstate;
    }

    rightGripperState rightgripperstate = rightGripperState.closed;
    leftGripperState leftgripperstate = leftGripperState.closed;

    public void updateArm (double slidesPos, Odometry odometry, Gamepad gamepad1, Telemetry telemetry, Gamepad gamepad2){

        switch (armstateTarget){

            case collect:

                setClaws(clawClosed);
                setGripperState(Delivery.targetGripperState.closeBoth);

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeCollection.reset();

                CollectionMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - collectTopPivotPos) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setMainPivot(collectTopPivotPos);

                RotateClaw.setPosition(rotateCollect);

                break;
            case delivery:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition()-deliverySecondPivot)*180)*timePerDegree, (Math.abs(getTopPivotPosition()-deliveryTopPivot)*180)*timePerDegree);

                setClaws(clawClosed);
                setGripperState(Delivery.targetGripperState.closeBoth);

                setSecondPivot(deliverySecondPivot);

                setMainPivot(deliveryTopPivot);

                RotateClaw.setPosition(rotateCollect);

                break;
            case deliverAuto:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMovingAuto = true;

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition()-deliverySecondPivotAuto)*180)*timePerDegree, (Math.abs(getTopPivotPosition()-deliveryTopPivotAuto)*180)*timePerDegree);

                setClaws(clawClosed);
                setGripperState(Delivery.targetGripperState.closeBoth);

                setSecondPivot(deliverySecondPivotAuto);

                setMainPivot(deliveryTopPivotAuto);

                RotateClaw.setPosition(rotateCollect);

                break;
            case moving:

                break;
            default:
        }

        switch (armstateCurrent){
            case moving:

                if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery){
                    armstateCurrent = armState.delivery;
                    DeliveryMoving = false;
                }

                if (DeliveryMovingAuto && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery){
                    armstateCurrent = armState.deliverAuto;
                    DeliveryMovingAuto = false;
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection){
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                }

                break;

            case delivery:

                if (slidesPos > 200) {

                    odometry.update();

                    if (gamepad2.dpad_right || gamepad1.dpad_right && mainPivotRight.getPosition() < lowdeliveryTopPivot) {
                        mainPivotOffSet = mainPivotOffSet + 0.005;
                    }

                    if (gamepad2.dpad_right || gamepad1.dpad_left && mainPivotRight.getPosition() > deliveryTopPivot) {
                        mainPivotOffSet = mainPivotOffSet - 0.005;
                    }

                    targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;
                    setMainPivot(targetMainPivot);
                    setSecondPivot(deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

                    double heading = odometry.heading;

                    if (heading > 270){
                        heading = heading-360;
                    } else if (heading > 90 && heading < 270) {
                        heading = 90;
                    }

                    targetRotatePos = 0.5 + (heading / 180);

                    secondRotate.setPosition(targetRotatePos);

                }

                break;
            default:
        }

        odometry.update();

        telemetry.addData("heading", odometry.heading);
        telemetry.addData("time", pivotMoveTimeDelivery.milliseconds());
        telemetry.addData("time to wait", timeToWaitDelivery);
    }


    public void updateGrippers (){

        switch (targetgripperState){

            case openBoth:
                setClaws(clawOpen);
                break;
            case closeBoth:
                setClaws(clawClosed);
                break;
            case openLeft:
                LeftClaw.setPosition(clawOpen);
                break;
            case openRight:
                RightClaw.setPosition(clawOpen);
                break;
            case closeLeft:
                LeftClaw.setPosition(clawClosed);
                break;
            case closeRight:
                RightClaw.setPosition(clawClosed);
                break;
            default:
        }

        if (RightClaw.getPosition() > 0.2){
            rightgripperstate = rightGripperState.open;
        } else if (RightClaw.getPosition() < 0.1) {
            rightgripperstate = rightGripperState.closed;
        }

        if (LeftClaw.getPosition() > 0.2){
            leftgripperstate = leftGripperState.open;
        } else if (LeftClaw.getPosition() < 0.1) {
            leftgripperstate = leftGripperState.closed;
        }

    }

    public void init(HardwareMap hardwareMap){
        hmap = hardwareMap;

        pivotMoveTimeDelivery.reset();
        pivotMoveTimeCollection.reset();

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

        secondRotate.setPwmRange(new PwmControl.PwmRange(650, 2400));

        setMainPivot(0);

        secondRotate.setPosition(0.5);

        setSecondPivot(collectSecondPivot);

        RotateClaw.setPosition(0.5);

        RightClaw.setPosition(clawClosed);
        LeftClaw.setPosition(clawClosed);

    }

    public double getSecondPivotPosition(){
        return secondPivotLeft.getPosition() + secondPivotRight.getPosition()/2;
    }

    public double getMainPivotPosition(){
        return (mainPivotLeft.getPosition() + mainPivotRight.getPosition()) / 2;
    }

    private void setMainPivot(double position){
        mainPivotLeft.setPosition(position);
        mainPivotRight.setPosition(position);
    }

    public double getTopPivotPosition(){
        return secondPivotLeft.getPosition();
    }

    private void setClaws(double position){
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

    public void setGripperState(targetGripperState gripperstate) {
        this.targetgripperState = gripperstate;
    }

    public armState getArmState() {
        return armstateCurrent;
    }



}
