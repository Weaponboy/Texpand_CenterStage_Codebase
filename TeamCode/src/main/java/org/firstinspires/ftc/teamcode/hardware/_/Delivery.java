package org.firstinspires.ftc.teamcode.hardware._;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Objects;


public class Delivery {

    ServoImplEx secondPivotLeft;
    ServoImplEx secondPivotRight;

    ServoImplEx RightClaw;
    ServoImplEx LeftClaw;

    ServoImplEx RotateClaw;

    ServoImplEx mainPivotLeft;
    ServoImplEx mainPivotRight;

    ServoImplEx secondRotate;

    ElapsedTime pivotMoveTimeDelivery = new ElapsedTime();
    ElapsedTime pivotMoveTimeAuto = new ElapsedTime();
    ElapsedTime pivotMoveTimeCollection = new ElapsedTime();

    Sensors sensors = new Sensors();

    double timeToWaitDelivery;
    double timeToWaitCollection;

    double timePerDegree = 7;

    double collectTopPivotPos = 0;
    double intermediateTopPivot = 0.3;
    double deliveryTopPivot = 0.7;
    double deliveryTopPivotNew = 1;
    double safeTopPivot = 0.3;

    double deliveryTopPivotAuto = 0.6;
    double deliverySecondPivotAuto = 0.45;
    double distancecalc;
    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.88;
    double deliverySecondPivot = -0.18;
    double lowdeliveryTopPivot = 1;

    double clawOpenDeliver = 0.35;
    double clawOpen = 0.25;
    double clawClosed = 0;

    double rotateCollect = 0.55;
    double rotateDeliver = 0.55;
    double rotateRight = 1;

    double secondRotateMiddle = 0.5;
    double secondRotateMiddleCollect = 0.5;

    double targetRotatePos;
    static final double adjustFactor = 1.1;
    boolean closeToCollection;

    public void setMainPivotOffSet(double mainPivotOffSet) {
        this.mainPivotOffSet = mainPivotOffSet;
    }
    boolean firtloop = true;
    double mainPivotOffSet = 0;
    double targetMainPivot = 0;
    static final double servoPosPerTick = 0.00004100;
    static final double mainToSecondConst = 0.5/0.3;
    static final double mainservopospermm = 0.0126;
    static final double mindistancemm = 12;
    static final double maxdistancemm = 28;

    boolean DeliveryMovingAuto = false;
    boolean DeliveryMoving = false;
    boolean CollectionMoving = false;
    boolean intermediateMoving = false;

    double deliveryIncrement = 0.01;

    HardwareMap hmap;

    public enum armState {
        delivery,
        deliverAuto,
        collect,
        moving,
        intermediate
    }

    public enum GripperState{
        closed,
        openDeliver,
        open
    }

    public enum leftGripperState{
        closed,
        openDeliver,
        open
    }

    public enum rightGripperState{
        closed,
        openDeliver,
        open
    }

    armState armstateTarget = armState.moving;
    armState armstateCurrent = armState.collect;

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

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeCollection.reset();

                CollectionMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setMainPivot(collectTopPivotPos);

                RotateClaw.setPosition(rotateCollect);

                break;
            case intermediate:

                setClaws(clawClosed);
                setGripperState(GripperState.closed);

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeCollection.reset();

                intermediateMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setMainPivot(intermediateTopPivot);

                RotateClaw.setPosition(rotateCollect);

                break;
            case delivery:

                mainPivotOffSet = 0.25;

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                setClaws(clawClosed);
                setGripperState(GripperState.closed);

                double distance = sensors.backBoard.getDistance(DistanceUnit.CM);

                if(distance > mindistancemm && distance < maxdistancemm){
                    mainPivotOffSet = 0.174 + (( distance - mindistancemm) * mainservopospermm);
                }

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                targetMainPivot = Range.clip(targetMainPivot, 0,1);

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition() - targetMainPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst)) * 180) * timePerDegree);


                setMainPivot(targetMainPivot);

                setSecondPivot(deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

                RotateClaw.setPosition(rotateDeliver);

                break;
            case deliverAuto:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMovingAuto = true;

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition()-deliverySecondPivotAuto)*180)*timePerDegree, (Math.abs(getTopPivotPosition()-deliveryTopPivotAuto)*180)*timePerDegree);

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                setSecondPivot(deliverySecondPivotAuto);

                setMainPivot(deliveryTopPivotAuto);

                RotateClaw.setPosition(rotateDeliver);

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

                if (intermediateMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection){
                    armstateCurrent = armState.intermediate;
                    armstateTarget = armState.collect;
                    intermediateMoving = false;
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= (timeToWaitCollection/2)){
                    setClaws(clawClosed);

                    setGripperState(GripperState.closed);
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection){
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                }

                break;

            case delivery:

//                if(firtloop){
//                    double distance = sensors.backBoard.getDistance(DistanceUnit.CM);
//                    if(distance > mindistancemm && distance < maxdistancemm){
//                        mainPivotOffSet = 0.2 + (( distance - mindistancemm) * mainservopospermm);
//                    }
//
//                    firtloop = false;
//                }

                odometry.update();

                if (gamepad2.dpad_left && mainPivotRight.getPosition() < lowdeliveryTopPivot) {
                    mainPivotOffSet = mainPivotOffSet + deliveryIncrement;
                }

                if (gamepad2.dpad_right && mainPivotRight.getPosition() > deliveryTopPivot-0.04) {
                    mainPivotOffSet = mainPivotOffSet - deliveryIncrement;
                }

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;
                setMainPivot(targetMainPivot);
                setSecondPivot(deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

                double heading = odometry.heading;

                if (odometry.heading > 180 && odometry.heading < 270) {
                    heading = odometry.heading - 180;
                } else if (odometry.heading > 90 && odometry.heading < 180) {
                    heading = odometry.heading - 180;
                } else if (odometry.heading > 270) {
                    heading = 90;
                }else if (odometry.heading < 90) {
                    heading = -90;
                }

                targetRotatePos = 0.5 + (heading / 180);

                secondRotate.setPosition(targetRotatePos);

            case intermediate:

                if (slidesPos < 50){
                    setArmTargetState(armState.collect);
                }

                break;
            default:
        }

        odometry.update();

    }

    public void updateArm (double slidesPos){

        switch (armstateTarget){

            case collect:

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeCollection.reset();

                CollectionMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setMainPivot(collectTopPivotPos);

                setClaws(clawClosed);

                RotateClaw.setPosition(rotateCollect);

                break;
            case intermediate:

                setClaws(clawClosed);
                setGripperState(GripperState.closed);

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeCollection.reset();

                intermediateMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setMainPivot(intermediateTopPivot);

                RotateClaw.setPosition(rotateCollect);

                break;
            case delivery:

                mainPivotOffSet = 0.3;

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                setClaws(clawClosed);
                setGripperState(GripperState.closed);

                double distance = sensors.backBoard.getDistance(DistanceUnit.CM);

                if(distance > mindistancemm && distance < maxdistancemm){
                    mainPivotOffSet = 0.23 + (( distance - mindistancemm) * mainservopospermm);
                }

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                targetMainPivot = Range.clip(targetMainPivot, 0,1);

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition() - targetMainPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst)) * 180) * timePerDegree);

                setMainPivot(targetMainPivot);
                setSecondPivot(deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

                RotateClaw.setPosition(rotateDeliver);

                break;
            case deliverAuto:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeAuto.reset();

                DeliveryMovingAuto = true;

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition()-deliverySecondPivotAuto)*180)*timePerDegree, (Math.abs(getTopPivotPosition()-deliveryTopPivotAuto)*180)*timePerDegree);

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                setSecondPivot(deliverySecondPivotAuto);

                setMainPivot(deliveryTopPivotAuto);

                RotateClaw.setPosition(rotateDeliver);

                break;
            case moving:

                break;
            default:
        }

        if (Objects.requireNonNull(armstateCurrent) == armState.moving) {

            if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery) {
                armstateCurrent = armState.delivery;
                DeliveryMoving = false;
            }

            if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                armstateCurrent = armState.deliverAuto;
                DeliveryMovingAuto = false;
            }

            if (intermediateMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                armstateCurrent = armState.intermediate;
                intermediateMoving = false;
            }

            if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                armstateCurrent = armState.collect;
                CollectionMoving = false;
            }
        }

    }

    public void updateGrippers (){

        switch (rightgripperstate){
            case open:
                RightClaw.setPosition(clawOpen);
                break;
            case closed:
                RightClaw.setPosition(clawClosed);
                break;
            case openDeliver:
                RightClaw.setPosition(clawOpenDeliver);
                break;
            default:
        }

        switch (leftgripperstate){
            case open:
                LeftClaw.setPosition(clawOpen);
                break;
            case closed:
                LeftClaw.setPosition(clawClosed);
                break;
            case openDeliver:
                LeftClaw.setPosition(clawOpenDeliver);
                break;
            default:
        }

    }

    public void init(HardwareMap hardwareMap){
        hmap = hardwareMap;

        pivotMoveTimeDelivery.reset();
        pivotMoveTimeCollection.reset();

        sensors.init(hardwareMap);

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

        RightClaw.setPwmRange(new PwmControl.PwmRange(900, 1900));
        LeftClaw.setPwmRange(new PwmControl.PwmRange(1200, 2300));

        mainPivotLeft = hardwareMap.get(ServoImplEx.class, "leftmain");
        mainPivotRight = hardwareMap.get(ServoImplEx.class, "rightmain");

        mainPivotLeft.setDirection(Servo.Direction.REVERSE);

        mainPivotLeft.setPwmRange(new PwmControl.PwmRange(600, 2500));
        mainPivotRight.setPwmRange(new PwmControl.PwmRange(600, 2500));

        RotateClaw = hardwareMap.get(ServoImplEx.class, "ClawRotate");

        RotateClaw.setDirection(Servo.Direction.FORWARD);

        RotateClaw.setPwmRange(new PwmControl.PwmRange(700, 2300));

        secondRotate = hardwareMap.get(ServoImplEx.class, "secondRotate");

        secondRotate.setPwmRange(new PwmControl.PwmRange(600, 2500));

        setMainPivot(0);

        secondRotate.setPosition(secondRotateMiddleCollect);

        setSecondPivot(collectSecondPivot);

        RotateClaw.setPosition(rotateCollect);

        RightClaw.setPosition(clawClosed);
        LeftClaw.setPosition(clawClosed);

    }

    public void setRotateClaw(double position){
        RotateClaw.setPosition(position);
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

    public void setGripperState(GripperState gripperstate) {
        switch (gripperstate){
            case open:
                this.rightgripperstate = rightGripperState.open;
                this.leftgripperstate = leftGripperState.open;
                break;
            case closed:
                this.rightgripperstate = rightGripperState.closed;
                this.leftgripperstate = leftGripperState.closed;
                break;
            case openDeliver:
                this.rightgripperstate = rightGripperState.openDeliver;
                this.leftgripperstate = leftGripperState.openDeliver;
                break;
            default:
        }

    }

    public void setRightGripperState(rightGripperState gripperstate) {
        this.rightgripperstate = gripperstate;
    }

    public void setLeftGripperState(leftGripperState gripperstate) {
        this.leftgripperstate = gripperstate;
    }

    public armState getArmState() {
        return armstateCurrent;
    }



}
