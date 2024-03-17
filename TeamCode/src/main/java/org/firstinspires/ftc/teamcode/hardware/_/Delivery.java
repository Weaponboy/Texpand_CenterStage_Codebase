package org.firstinspires.ftc.teamcode.hardware._;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Constants_and_Setpoints.RobotArm;

import java.util.Objects;


public class Delivery {

    ServoImplEx secondPivotLeft;
    ServoImplEx secondPivotRight;

    ServoImplEx RightClaw;
    ServoImplEx LeftClaw;

    ServoImplEx RotateClaw;
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
    double deliverySecondPivotAuto = 0;
    double distancecalc;
    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.84;
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
    double ArmExtensionHome = 0.96;
    double deliveryMainIncrement = 0.015;
    double deliveryArmIncrement = 0.02;

    double armRotateToMainConstLEFT = 0.0944;
    double armRotateToSecondRotateConstLEFT = -1.54;
    double armRotateToSecondPivotConstLEFT = 0.278;
    double armRotateToArmExtendConstLEFT = -2.494;

    double armRotateToMainConstRIGHT = -0.0944;
    double armRotateToSecondRotateConstRIGHT = -1.52;
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
        readyToDrop
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
        whiteBlue,
        whiteRed,
        yellow1Blue,
        yellow2Blue,
        yellow3Blue,
        yellow1Red,
        yellow2Red,
        yellow3Red
    }

    RobotArm whiteBlue = new RobotArm(0.916,0.5117, 0.13, 0.256, rotateCollect, 1000, 0);

    RobotArm whiteRed = new RobotArm(0.916,0.5117, 0.13, 0.256, rotateCollect, 1000, 0);

    RobotArm yellow1Blue = new RobotArm(0.916, ArmPositionMid, 0.46, 0.316, 0.94, 800, 0);
    RobotArm yellow2Blue = new RobotArm(0.916, ArmPositionMid, secondRotateMiddle, 0.256, rotateCollect, 800, 0);
    RobotArm yellow3Blue = new RobotArm(0.916, ArmPositionMid, secondRotateMiddle, 0.256, rotateCollect, 800, 0);

    RobotArm yellow1Red = new RobotArm(0.916, ArmPositionMid, 0.46, 0.316, 0.94, 800, 0);
    RobotArm yellow2Red = new RobotArm(0.916, ArmPositionMid, secondRotateMiddle, 0.256, rotateCollect, 800, 0);
    RobotArm yellow3Red = new RobotArm(0.916, ArmPositionMid, secondRotateMiddle, 0.256, rotateCollect, 800, 0);

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

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                RotateArm.setPosition(ArmPositionMid);

                ArmExtension.setPosition(ArmExtensionHome);

                pivotMoveTimeCollection.reset();

                CollectionMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getMainPivotPosition() - collectTopPivotPos) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setMainPivot(collectTopPivotPos);

                setClaws(clawClosed);

                RotateClaw.setPosition(rotateCollect);

                break;
            case delivery:

                if (RotateArm.getPosition() < 0.48 && RotateArm.getPosition() > 0.52){
                    secondRotate.setPosition(secondRotateMiddle);
                }

                if (!PivotSet){
                    pivotMoveTimeDelivery.reset();

                    DeliveryMoving = true;

                    PivotSet = true;

                    timeToWaitDelivery = Math.max((Math.abs(getMainPivotPosition() - getMainPivotSetPoint(pixelPlacement)) * 180) * 4, (Math.abs(getSecondPivotPosition() - getSecondPivotSetPoint(pixelPlacement)) * 180) * 4);
                }

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                if (pivotMoveTimeDelivery.milliseconds() > (timeToWaitDelivery) && DeliveryMoving) {
                    armstateCurrent = armState.readyToDrop;

                    armstateTarget = armState.readyToDrop;

                    PivotSet = false;

                    DeliveryMoving = false;

                }

                double distance = 310 - odometry.X;

                ArmExtension.setPosition(1-distance*0.04);

                setMainPivot(getMainPivotSetPoint(pixelPlacement));

                setSecondPivot(getSecondPivotSetPoint(pixelPlacement));

                RotateClaw.setPosition(getRotateClawSetPoint(pixelPlacement));

                break;
            case deliverAuto:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeAuto.reset();

                DeliveryMovingAuto = true;

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition()-deliverySecondPivotAuto)*180)*timePerDegree, (Math.abs(getTopPivotPosition()-0.85)*180)*timePerDegree);

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                setSecondPivot(deliverySecondPivotAuto);

                setMainPivot(0.85);

                targetMainPivot = 0.85 - slidesPos * servoPosPerTick + mainPivotOffSet;

                RotateClaw.setPosition(rotateDeliver);

                break;
            case droppingWhites:

                if(!PivotSet){

                    secondRotate.setPosition(secondRotateMiddle);

                    pivotTimer.reset();

                    PivotSet = true;

                    numberOfDegreesToTarget = (Math.abs(getMainPivotPosition()-getMainPivotSetPoint(pixelPlacement))*180);

                }

                if (pivotTimer.milliseconds() > (numberOfDegreesToTarget) * timePerDegree) {

                    secondRotate.setPosition(getSecondRotateSetPoint(pixelPlacement));

                    RotateArm.setPosition(getArmRotateSetPoint(pixelPlacement));

                    RotateClaw.setPosition(getRotateClawSetPoint(pixelPlacement));

                    startExtending = true;

                }

                if (startExtending){

                    ArmExtension.setPosition(ArmExtension.getPosition() - 0.003);

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

            if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                armstateCurrent = armState.deliverAuto;
                DeliveryMovingAuto = false;
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

        mainPivotLeft.setPwmRange(new PwmControl.PwmRange(600, 2500));
        mainPivotRight.setPwmRange(new PwmControl.PwmRange(600, 2500));

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

        if (pixelTarget == PixelsAuto.whiteBlue){
            return whiteBlue.getMainPivot();
        } else if (pixelTarget == PixelsAuto.whiteRed) {
            return whiteRed.getMainPivot();
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

        if (pixelTarget == PixelsAuto.whiteBlue){
            return whiteBlue.getSecondPivot();
        } else if (pixelTarget == PixelsAuto.whiteRed) {
            return whiteRed.getSecondPivot();
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

        if (pixelTarget == PixelsAuto.whiteBlue){
            return whiteBlue.getSecondRotate();
        } else if (pixelTarget == PixelsAuto.whiteRed) {
            return whiteRed.getSecondRotate();
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

        if (pixelTarget == PixelsAuto.whiteBlue){
            return whiteBlue.getArmRotate();
        } else if (pixelTarget == PixelsAuto.whiteRed) {
            return whiteRed.getArmRotate();
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

        if (pixelTarget == PixelsAuto.whiteBlue){
            return whiteBlue.getClawRotate();
        } else if (pixelTarget == PixelsAuto.whiteRed) {
            return whiteRed.getClawRotate();
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



}