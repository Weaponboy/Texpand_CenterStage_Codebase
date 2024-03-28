package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.Constants_and_Setpoints.RobotArm;

import java.util.Objects;


public class Delivery {

    ServoImplEx secondPivotLeft;
    ServoImplEx secondPivotRight;

    ServoImplEx RightClaw;
    ServoImplEx LeftClaw;

    public ServoImplEx RotateClaw;
    public Servo RotateArm;
    public ServoImplEx ArmExtension;
    ServoImplEx mainPivotLeft;
    ServoImplEx mainPivotRight;

    public double getSecondRotatePosition() {
        return secondRotate.getPosition();
    }

    public void setSecondRotate(double setpoint) {
        secondRotate.setPosition(setpoint);
    }

    ServoImplEx secondRotate;

    ElapsedTime pivotMoveTimeDelivery = new ElapsedTime();
    ElapsedTime pivotMoveTimeAuto = new ElapsedTime();
    ElapsedTime pivotMoveTimeCollection = new ElapsedTime();

    ElapsedTime pivotTimer = new ElapsedTime();

    Sensors sensors = new Sensors();

    double timeToWaitDelivery;
    double timeToWaitCollection;
    double timeToWaitSideMove;

    double numberOfDegreesToTarget;

    double timePerDegree = 5;

    public double ArmPositionMid = 0.5;
    double collectTopPivotPos = 0.188;
    double intermediateTopPivot = 0.3;
    double deliveryTopPivot = 0.7;
    double deliveryTopPivotNew = 1;
    double safeTopPivot = 0.3;

    double deliveryTopPivotAuto = 0.7;
    double deliverySecondPivotAuto = 0.2;
    double distancecalc;
    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.8;
    double deliverySecondPivot = -0.1;
    double lowdeliveryTopPivot = 1;

    double clawOpenDeliver = 0.6;
    double clawOpen = 0.6;
    double clawClosed = 1;

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
    double mainPivotVertOffSet = 0;
    double targetMainPivot = 0;
    static final double servoPosPerTick = 0.00004100;
    static final double mainToSecondConst = 0.5/0.3;
    static final double mainservopospermm = 0.0126;
    static final double mindistancemm = 8;
    static final double maxdistancemm = 20;
    static final double distanceToPivot = 0.1995;

    boolean PivotSet = false;

    boolean DeliveryMovingAuto = false;
    boolean DeliveryMoving = false;
    boolean CollectionMoving = false;
    public double ArmExtensionHome = 0.92;
    double deliveryMainIncrement = 0.015;
    double deliveryArmIncrement = 0.02;

    double armRotateToMainConstLEFT = 0.0944;
    double armRotateToSecondRotateConstLEFT = -2.54;
    double armRotateToSecondPivotConstLEFT = 0.278;
    double armRotateToArmExtendConstLEFT = -2.494;

    double armRotateToMainConstRIGHT = -0.0944;
    double armRotateToSecondRotateConstRIGHT = -2.52;
    double armRotateToSecondPivotConstRIGHT = -0.278;
    double armRotateToArmExtendConstRIGHT = 2.494;
    double Mainpivottoextendconst = -10.89;
    double Mainpivottosecondconst = 1.45;

    boolean movingCenter = false;

    HardwareMap hmap;

    public enum armState {
        delivery,
        deliverAuto,
        collect,
        moving,
        droppingWhites,
        readyToDrop,
        readyForDelivering
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

    public enum PixelsAuto{
        backboardRight,
        backboardLeft,
        yellow1Blue,
        yellow2Blue,
        yellow3Blue,
        yellow1Red,
        yellow2Red,
        yellow3Red
    }

    RobotArm backboardRight = new RobotArm(0.926,0.62, 0.1, 0.160, 0.15, -1, -1);

    RobotArm backboardLeft = new RobotArm(0.926,0.35, 0.85, 0.174, 0.95, -1, -1);

    RobotArm yellow1Blue = new RobotArm(0.956, ArmPositionMid, 0.55, 0.194, 0.94, ArmExtensionHome, -1);
    RobotArm yellow2Blue = new RobotArm(0.956, ArmPositionMid, secondRotateMiddle, 0.186, rotateCollect, ArmExtensionHome, -1);
    RobotArm yellow3Blue = new RobotArm(0.956, ArmPositionMid, secondRotateMiddle, 0.216, rotateCollect, ArmExtensionHome, -1);

    RobotArm yellow1Red = new RobotArm(0.916, ArmPositionMid, 0.46, 0.316, 0.94, -1, -1);
    RobotArm yellow2Red = new RobotArm(0.916, ArmPositionMid, secondRotateMiddle, 0.256, rotateCollect, -1, -1);
    RobotArm yellow3Red = new RobotArm(0.916, ArmPositionMid, secondRotateMiddle, 0.256, rotateCollect, -1, -1);

    armState armstateTarget = armState.moving;

    armState armstateCurrent = armState.collect;

    boolean startExtending;

    public rightGripperState getRightgripperstate() {
        return rightgripperstate;
    }

    public leftGripperState getLeftgripperstate() {
        return leftgripperstate;
    }

    rightGripperState rightgripperstate = rightGripperState.closed;
    leftGripperState leftgripperstate = leftGripperState.closed;

    boolean sideMoving;

    public void updateArm (double slidesPos, Odometry odometry, Gamepad gamepad2){

        switch (armstateTarget){

            case collect:

                RotateArm.setPosition(ArmPositionMid);

                ArmExtension.setPosition(ArmExtensionHome);

                if (RotateArm.getPosition() > 0.46){
                    timeToWaitSideMove = (Math.abs(RotateArm.getPosition() - ArmPositionMid) * 180) * 5;
                } else if (RotateArm.getPosition() < 0.46) {
                    timeToWaitSideMove = (Math.abs(RotateArm.getPosition() - ArmPositionMid) * 180) * 10;
                }

                mainPivotOffSet = 0;

                mainPivotVertOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeCollection.reset();

                movingCenter = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                RotateClaw.setPosition(rotateCollect);

                break;

            case delivery:

                mainPivotVertOffSet = 0;

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                targetMainPivot = Range.clip(targetMainPivot, 0,1);

                setMainPivot(targetMainPivot);

                setSecondPivot(deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition() - targetMainPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst)) * 180) * timePerDegree);

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

                if (movingCenter && pivotMoveTimeCollection.milliseconds() >= timeToWaitSideMove){
                    pivotMoveTimeCollection.reset();
                    RotateArm.setPosition(ArmPositionMid);
                    CollectionMoving = true;
                    movingCenter = false;
                    setMainPivot(collectTopPivotPos);
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection){
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                    RotateArm.setPosition(ArmPositionMid);
                }

                break;

            case delivery:

                odometry.update();

                if (gamepad2.dpad_left) {
                    mainPivotOffSet = mainPivotOffSet + deliveryMainIncrement;

                } else if (gamepad2.dpad_right) {
                    mainPivotOffSet = mainPivotOffSet - deliveryMainIncrement;
                }

//                if (gamepad2.left_stick_y < -0.1){
//                    mainPivotVertOffSet = mainPivotVertOffSet + gamepad2.left_stick_y/100;
//                }else if (gamepad2.left_stick_y > 0.1){
//                    mainPivotVertOffSet = mainPivotVertOffSet + gamepad2.left_stick_y/100;
//                }

                if (gamepad2.right_stick_x > 0.05 && RotateArm.getPosition() > 0.3 ) {
                    RotateArm.setPosition(RotateArm.getPosition() - gamepad2.right_stick_x/80);
                }else if (gamepad2.right_stick_x < -0.05 && RotateArm.getPosition() < 0.66 ) {
                    RotateArm.setPosition(RotateArm.getPosition() - gamepad2.right_stick_x/80);
                }

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                if (RotateArm.getPosition() > ArmPositionMid) {
                    secondRotate.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondRotateConstLEFT + secondRotateMiddle);
                    setMainPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToMainConstLEFT + targetMainPivot + mainPivotVertOffSet);
                    setSecondPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondPivotConstLEFT + (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst) + (mainPivotVertOffSet * Mainpivottosecondconst));
                    ArmExtension.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToArmExtendConstLEFT + (mainPivotVertOffSet*Mainpivottoextendconst) + ArmExtensionHome);
                } else {
                    secondRotate.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondRotateConstRIGHT + secondRotateMiddle);
                    setMainPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToMainConstRIGHT + targetMainPivot + mainPivotVertOffSet);
                    setSecondPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondPivotConstRIGHT + (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst) + (mainPivotVertOffSet * Mainpivottosecondconst));
                    ArmExtension.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToArmExtendConstRIGHT + (mainPivotVertOffSet*Mainpivottoextendconst) + ArmExtensionHome);
                }

//                odometry.update();
//
//                double heading = odometry.heading;
//
//                if (odometry.heading > 180 && odometry.heading < 270) {
//                    heading = odometry.heading - 180;
//                } else if (odometry.heading > 90 && odometry.heading < 180) {
//                    heading = odometry.heading - 180;
//                } else if (odometry.heading > 270) {
//                    heading = 90;
//                } else if (odometry.heading < 90) {
//                    heading = -90;
//                }
//
//                targetRotatePos = 0.5 + (heading / 180);
//
//                secondRotate.setPosition(targetRotatePos);
                break;
            default:

        }

    }

    public void updateArm (double slidesPos, boolean touchingBackdrop, PixelsAuto pixelPlacement, Odometry odometry){

        switch (armstateTarget){

            case collect:

                if (RotateArm.getPosition() > 0.46 && !sideMoving && !CollectionMoving){

                    timeToWaitSideMove = (Math.abs(RotateArm.getPosition() - ArmPositionMid) * 180) * 15;

                    secondRotate.setPosition(secondRotateMiddleCollect);

                    RotateArm.setPosition(ArmPositionMid);

                    ArmExtension.setPosition(1);

                    setSecondPivot(collectSecondPivot);

                    RotateClaw.setPosition(rotateCollect);

                    pivotMoveTimeCollection.reset();

                    sideMoving = true;

                } else if (RotateArm.getPosition() < 0.46 && !sideMoving && !CollectionMoving) {

                    timeToWaitSideMove = (Math.abs(RotateArm.getPosition() - ArmPositionMid) * 180) * 15;

                    secondRotate.setPosition(secondRotateMiddleCollect);

                    RotateArm.setPosition(ArmPositionMid);

                    ArmExtension.setPosition(1);

                    setSecondPivot(collectSecondPivot);

                    RotateClaw.setPosition(rotateCollect);

                    pivotMoveTimeCollection.reset();

                    sideMoving = true;
                }else {

                    if (!CollectionMoving){

                        pivotMoveTimeCollection.reset();

                        timeToWaitCollection = (Math.abs(getMainPivotPosition() - collectTopPivotPos) * 180) * timePerDegree+2;

                        CollectionMoving = true;

                        secondRotate.setPosition(secondRotateMiddleCollect);

                        RotateArm.setPosition(ArmPositionMid);

                        ArmExtension.setPosition(1);

                        setSecondPivot(collectSecondPivot);

                        setMainPivot(collectTopPivotPos);

                        RotateClaw.setPosition(rotateCollect);

                    }

                }

                if(CollectionMoving && pivotMoveTimeCollection.milliseconds() > timeToWaitCollection){
                    armstateCurrent = armState.collect;
                    armstateTarget = armState.moving;
                    ArmExtension.setPosition(ArmExtensionHome);
                    CollectionMoving = false;
                }

                if(sideMoving && pivotMoveTimeCollection.milliseconds() > timeToWaitSideMove){
                    sideMoving = false;
                    pivotMoveTimeCollection.reset();
                    timeToWaitCollection = (Math.abs(getMainPivotPosition() - collectTopPivotPos) * 180) * timePerDegree;
                    setMainPivot(collectTopPivotPos);
                    CollectionMoving = true;
                }

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                break;
            case readyForDelivering:

                secondRotate.setPosition(secondRotateMiddle);

                RotateArm.setPosition(ArmPositionMid);

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                ArmExtension.setPosition(ArmExtensionHome);

                setMainPivot(0.82);

                setSecondPivot(deliverySecondPivot);

                RotateClaw.setPosition(rotateDeliver);

                break;
            case delivery:

//                if (RotateArm.getPosition() < 0.48 && RotateArm.getPosition() > 0.52){
//                    secondRotate.setPosition(secondRotateMiddle);
//                }

                if (!PivotSet){
                    pivotMoveTimeDelivery.reset();

                    DeliveryMoving = true;

                    PivotSet = true;

                    timeToWaitDelivery = Math.max(Math.abs(getMainPivotPosition() - getMainPivotSetPoint(pixelPlacement) * 180) * 5, Math.abs(ArmExtension.getPosition() - getExtensionSetPoint(pixelPlacement))*6);
                }

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                if (pivotMoveTimeDelivery.milliseconds() > (timeToWaitDelivery) && DeliveryMoving) {

                    armstateCurrent = armState.readyToDrop;

                    armstateTarget = armState.readyToDrop;

                    PivotSet = false;

                    DeliveryMoving = false;

                } else if (touchingBackdrop) {

                    armstateCurrent = armState.readyToDrop;

                    armstateTarget = armState.readyToDrop;

                    PivotSet = false;

                    DeliveryMoving = false;
                }

                ArmExtension.setPosition(getExtensionSetPoint(pixelPlacement));

                setMainPivot(getMainPivotSetPoint(pixelPlacement));

                setSecondPivot(getSecondPivotSetPoint(pixelPlacement));

                RotateClaw.setPosition(getRotateClawSetPoint(pixelPlacement));

                break;

            case deliverAuto:

                timeToWaitDelivery = (Math.abs(getMainPivotPosition()-getMainPivotSetPoint(pixelPlacement))*180)*timePerDegree;

                setMainPivot(getMainPivotSetPoint(pixelPlacement));

//                ArmExtension.setPosition(0.8);

                setGripperState(GripperState.closed);

                setSecondPivot(getSecondPivotSetPoint(pixelPlacement));

                setClaws(clawClosed);

                pivotMoveTimeAuto.reset();

                DeliveryMovingAuto = true;

                armstateTarget = armState.moving;

                break;
            case droppingWhites:

                if(!PivotSet){

                    pivotTimer.reset();

                    PivotSet = true;

                    numberOfDegreesToTarget = (Math.abs(getMainPivotPosition()-getMainPivotSetPoint(pixelPlacement))*180);

                }

                if (pivotTimer.milliseconds() > (numberOfDegreesToTarget) * timePerDegree) {

                    RotateArm.setPosition(getArmRotateSetPoint(pixelPlacement));

                    RotateClaw.setPosition(getRotateClawSetPoint(pixelPlacement));

                    startExtending = true;

                }

                if (startExtending){

                    ArmExtension.setPosition(ArmExtension.getPosition() - 0.04);

                    if (ArmExtension.getPosition() < 0.39) {

                        armstateCurrent = armState.readyToDrop;

                        armstateTarget = armState.readyToDrop;

                        PivotSet = false;

                    }

                    if (touchingBackdrop){
                        armstateCurrent = armState.readyToDrop;

                        armstateTarget = armState.readyToDrop;

                        PivotSet = false;
                    }

                }

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                setSecondPivot(getSecondPivotSetPoint(pixelPlacement));

                setMainPivot(getMainPivotSetPoint(pixelPlacement));

                secondRotate.setPosition(getSecondRotateSetPoint(pixelPlacement));

                break;
            default:
        }

        if (Objects.requireNonNull(armstateCurrent) == armState.moving) {

            if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery){
                armstateCurrent = armState.delivery;
                DeliveryMoving = false;
            }

            if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection){
                armstateCurrent = armState.collect;
                CollectionMoving = false;
            }

            if(DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() > timeToWaitDelivery/1.2){
                RotateArm.setPosition(getArmRotateSetPoint(pixelPlacement));
                RotateClaw.setPosition(getRotateClawSetPoint(pixelPlacement));
            }

            if(DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() > timeToWaitDelivery){
                ArmExtension.setPosition(0.4);
                DeliveryMovingAuto = false;
                armstateCurrent = armState.deliverAuto;
            }

        }

        if (Objects.requireNonNull(armstateTarget) == armState.moving) {

            if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery){
                armstateCurrent = armState.delivery;
                DeliveryMoving = false;
            }

            if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection){
                armstateCurrent = armState.collect;
                CollectionMoving = false;
            }

            if(DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() > (timeToWaitDelivery*0.7)){
                RotateArm.setPosition(getArmRotateSetPoint(pixelPlacement));
                RotateClaw.setPosition(getRotateClawSetPoint(pixelPlacement));
                secondRotate.setPosition(getSecondRotateSetPoint(pixelPlacement));
                ArmExtension.setPosition(0.4);
            }

            if(DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() > timeToWaitDelivery){
                DeliveryMovingAuto = false;
                armstateCurrent = armState.deliverAuto;
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

        RightClaw.setPwmRange(new PwmControl.PwmRange(600, 2500));
        LeftClaw.setPwmRange(new PwmControl.PwmRange(600, 2500));

        mainPivotLeft = hardwareMap.get(ServoImplEx.class, "leftmain");
        mainPivotRight = hardwareMap.get(ServoImplEx.class, "rightmain");

        mainPivotLeft.setDirection(Servo.Direction.REVERSE);

        mainPivotLeft.setPwmRange(new PwmControl.PwmRange(650, 2500));
        mainPivotRight.setPwmRange(new PwmControl.PwmRange(650, 2500));

        RotateClaw = hardwareMap.get(ServoImplEx.class, "ClawRotate");

        RotateClaw.setDirection(Servo.Direction.FORWARD);

        RotateClaw.setPwmRange(new PwmControl.PwmRange(700, 2300));

        secondRotate = hardwareMap.get(ServoImplEx.class, "secondRotate");

        secondRotate.setPwmRange(new PwmControl.PwmRange(600, 2500));

        RotateArm  = hardwareMap.get(Servo.class,"RotateArm");

        secondRotate.setPwmRange(new PwmControl.PwmRange(900, 2200));

        ArmExtension = hardwareMap.get(ServoImplEx.class, "ArmExtension");

        ArmExtension.setDirection(Servo.Direction.REVERSE);

        ArmExtension.setPwmRange(new PwmControl.PwmRange(600, 2400));

        ArmExtension.setPosition(ArmExtensionHome);

        RotateArm.setPosition(ArmPositionMid);

        setMainPivot(collectTopPivotPos);

        secondRotate.setPosition(secondRotateMiddleCollect);

        setSecondPivot(collectSecondPivot);

        RotateClaw.setPosition(rotateCollect);

        RightClaw.setPosition(clawOpen);
        LeftClaw.setPosition(clawOpen);

        setGripperState(GripperState.open);

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

    public void setMainPivot(double position){
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

    public void setSecondPivot(double position){
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

    public double getMainPivotSetPoint(PixelsAuto pixelTarget){

        if (pixelTarget == PixelsAuto.backboardRight){
            return backboardRight.getMainPivot();
        } else if (pixelTarget == PixelsAuto.backboardLeft) {
            return backboardLeft.getMainPivot();
        }else if (pixelTarget == PixelsAuto.yellow1Blue) {
            return yellow1Blue.getMainPivot();
        }else if (pixelTarget == PixelsAuto.yellow2Blue) {
            return yellow2Blue.getMainPivot();
        }else if (pixelTarget == PixelsAuto.yellow3Blue) {
            return yellow3Blue.getMainPivot();
        }else if (pixelTarget == PixelsAuto.yellow1Red) {
            return yellow1Red.getMainPivot();
        }else if (pixelTarget == PixelsAuto.yellow2Red) {
            return yellow2Red.getMainPivot();
        }else if(pixelTarget == PixelsAuto.yellow3Red) {
            return yellow3Red.getMainPivot();
        }else{
            return 0;
        }

    }

    public double getSecondPivotSetPoint(PixelsAuto pixelTarget){

        if (pixelTarget == PixelsAuto.backboardRight){
            return backboardRight.getSecondPivot();
        } else if (pixelTarget == PixelsAuto.backboardLeft) {
            return backboardLeft.getSecondPivot();
        }else if (pixelTarget == PixelsAuto.yellow1Blue) {
            return yellow1Blue.getSecondPivot();
        }else if (pixelTarget == PixelsAuto.yellow2Blue) {
            return yellow2Blue.getSecondPivot();
        }else if (pixelTarget == PixelsAuto.yellow3Blue) {
            return yellow3Blue.getSecondPivot();
        }else if (pixelTarget == PixelsAuto.yellow1Red) {
            return yellow1Red.getSecondPivot();
        }else if (pixelTarget == PixelsAuto.yellow2Red) {
            return yellow2Red.getSecondPivot();
        }else if(pixelTarget == PixelsAuto.yellow3Red) {
            return yellow3Red.getSecondPivot();
        }else{
            return 0;
        }

    }

    public double getSecondRotateSetPoint(PixelsAuto pixelTarget){

        if (pixelTarget == PixelsAuto.backboardRight){
            return backboardRight.getSecondRotate();
        } else if (pixelTarget == PixelsAuto.backboardLeft) {
            return backboardLeft.getSecondRotate();
        }else if (pixelTarget == PixelsAuto.yellow1Blue) {
            return yellow1Blue.getSecondRotate();
        }else if (pixelTarget == PixelsAuto.yellow2Blue) {
            return yellow2Blue.getSecondRotate();
        }else if (pixelTarget == PixelsAuto.yellow3Blue) {
            return yellow3Blue.getSecondRotate();
        }else if (pixelTarget == PixelsAuto.yellow1Red) {
            return yellow1Red.getSecondRotate();
        }else if (pixelTarget == PixelsAuto.yellow2Red) {
            return yellow2Red.getSecondRotate();
        }else if(pixelTarget == PixelsAuto.yellow3Red) {
            return yellow3Red.getSecondRotate();
        }else{
            return 0;
        }

    }

    public double getArmRotateSetPoint(PixelsAuto pixelTarget){

        if (pixelTarget == PixelsAuto.backboardRight){
            return backboardRight.getArmRotate();
        } else if (pixelTarget == PixelsAuto.backboardLeft) {
            return backboardLeft.getArmRotate();
        }else if (pixelTarget == PixelsAuto.yellow1Blue) {
            return yellow1Blue.getArmRotate();
        }else if (pixelTarget == PixelsAuto.yellow2Blue) {
            return yellow2Blue.getArmRotate();
        }else if (pixelTarget == PixelsAuto.yellow3Blue) {
            return yellow3Blue.getArmRotate();
        }else if (pixelTarget == PixelsAuto.yellow1Red) {
            return yellow1Red.getArmRotate();
        }else if (pixelTarget == PixelsAuto.yellow2Red) {
            return yellow2Red.getArmRotate();
        }else if(pixelTarget == PixelsAuto.yellow3Red) {
            return yellow3Red.getArmRotate();
        }else{
            return 0;
        }

    }

    public double getRotateClawSetPoint(PixelsAuto pixelTarget){

        if (pixelTarget == PixelsAuto.backboardRight){
            return backboardRight.getClawRotate();
        } else if (pixelTarget == PixelsAuto.backboardLeft) {
            return backboardLeft.getClawRotate();
        }else if (pixelTarget == PixelsAuto.yellow1Blue) {
            return yellow1Blue.getClawRotate();
        }else if (pixelTarget == PixelsAuto.yellow2Blue) {
            return yellow2Blue.getClawRotate();
        }else if (pixelTarget == PixelsAuto.yellow3Blue) {
            return yellow3Blue.getClawRotate();
        }else if (pixelTarget == PixelsAuto.yellow1Red) {
            return yellow1Red.getClawRotate();
        }else if (pixelTarget == PixelsAuto.yellow2Red) {
            return yellow2Red.getClawRotate();
        }else if(pixelTarget == PixelsAuto.yellow3Red) {
            return yellow3Red.getClawRotate();
        }else{
            return 0;
        }

    }

    public double getExtensionSetPoint(PixelsAuto pixelTarget){

        if (pixelTarget == PixelsAuto.backboardRight){
            return backboardRight.getArmExtension();
        } else if (pixelTarget == PixelsAuto.backboardLeft) {
            return backboardLeft.getArmExtension();
        }else if (pixelTarget == PixelsAuto.yellow1Blue) {
            return yellow1Blue.getArmExtension();
        }else if (pixelTarget == PixelsAuto.yellow2Blue) {
            return yellow2Blue.getArmExtension();
        }else if (pixelTarget == PixelsAuto.yellow3Blue) {
            return yellow3Blue.getArmExtension();
        }else if (pixelTarget == PixelsAuto.yellow1Red) {
            return yellow1Red.getArmExtension();
        }else if (pixelTarget == PixelsAuto.yellow2Red) {
            return yellow2Red.getArmExtension();
        }else if(pixelTarget == PixelsAuto.yellow3Red) {
            return yellow3Red.getArmExtension();
        }else{
            return 0;
        }

    }


}