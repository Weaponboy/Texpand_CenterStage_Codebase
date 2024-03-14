package org.firstinspires.ftc.teamcode.hardware._;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

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

    Sensors sensors = new Sensors();

    double timeToWaitDelivery;
    double timeToWaitCollection;
    double timeToWaitSideMove;

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
    double collectSecondPivot = 0.835;
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

    public void updateArm (double slidesPos){

        switch (armstateTarget){

            case collect:

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                RotateArm.setPosition(ArmPositionMid);

                ArmExtension.setPosition(ArmExtensionHome);

                timeToWaitSideMove = (Math.abs(RotateArm.getPosition() - ArmPositionMid) * 180) * 5;

                pivotMoveTimeCollection.reset();

                CollectionMoving = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setClaws(clawClosed);

                RotateClaw.setPosition(rotateCollect);

                break;
            case delivery:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                double distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                RobotLog.d("distance reading 1: " + distance);

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 2: " + distance);
                }

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 3: " + distance);
                }

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 4: " + distance);
                }

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 5: " + distance);
                }

                mainPivotOffSet = distanceToPivot + (( distance - mindistancemm) * mainservopospermm);

                RobotLog.d("mainPivotOffSet" + mainPivotOffSet);

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                targetMainPivot = Range.clip(targetMainPivot, 0,1);

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition() - targetMainPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst)) * 180) * timePerDegree);

                RobotLog.d("targetMainPivot" + targetMainPivot);

                setMainPivot(targetMainPivot);

                setSecondPivot(deliverySecondPivot + (targetMainPivot - deliveryTopPivot) * mainToSecondConst);

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

                if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.delivery;
                    DeliveryMoving = false;
                }

                if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.deliverAuto;
                    DeliveryMovingAuto = false;
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                }

                break;
            default:
        }

        if (Objects.requireNonNull(armstateCurrent) == armState.moving) {

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

            if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                armstateCurrent = armState.deliverAuto;
                DeliveryMovingAuto = false;
            }

        }

        if (Objects.requireNonNull(armstateCurrent) == armState.delivery){

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

        }


    }

    public void updateArm (double slidesPos, Odometry odometry, boolean backboardClearance){

        switch (armstateTarget){

            case collect:

                mainPivotOffSet = 0;

                secondRotate.setPosition(secondRotateMiddleCollect);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                RotateArm.setPosition(ArmPositionMid);

                ArmExtension.setPosition(ArmExtensionHome);

                timeToWaitSideMove = (Math.abs(RotateArm.getPosition() - ArmPositionMid) * 180) * 5;

                pivotMoveTimeCollection.reset();

                movingCenter = true;

                timeToWaitCollection = Math.max((Math.abs(getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - intermediateTopPivot) * 180) * timePerDegree);

                setSecondPivot(collectSecondPivot);

                setClaws(clawClosed);

                RotateClaw.setPosition(rotateCollect);

                break;
            case delivery:

                if (RotateArm.getPosition() > 0.48 && RotateArm.getPosition() < 0.44){
                    secondRotate.setPosition(secondRotateMiddle);
                }

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                double distance = (305 - odometry.X);

                if (backboardClearance){
                    mainPivotOffSet = (distanceToPivot-0.1) + (( distance - mindistancemm) * mainservopospermm);
                }else if (odometry.heading < 160){
                    mainPivotOffSet = (distanceToPivot+0.25) + (( distance - mindistancemm) * mainservopospermm);
                }else {
                    mainPivotOffSet = distanceToPivot + (( distance - mindistancemm) * mainservopospermm);
                }

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                targetMainPivot = Range.clip(targetMainPivot, 0,1);

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition() - targetMainPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst)) * 180) * timePerDegree);

                setMainPivot(targetMainPivot);

                setSecondPivot(deliverySecondPivot + (targetMainPivot - deliveryTopPivot) * mainToSecondConst);

                RotateClaw.setPosition(rotateDeliver);

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
            case moving:

                if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.delivery;
                    DeliveryMoving = false;
                }

                if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.deliverAuto;
                    DeliveryMovingAuto = false;
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                }

                break;
            default:
        }

        if (Objects.requireNonNull(armstateCurrent) == armState.moving) {

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

            if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                armstateCurrent = armState.deliverAuto;
                DeliveryMovingAuto = false;
            }

        }

        if (Objects.requireNonNull(armstateCurrent) == armState.deliverAuto){

            double distanceAccount = 0;

            if (odometry.X > 280 && odometry.X < 300){
                distanceAccount = -0.4;
            } else if (odometry.X > 300 && odometry.X < 305){
                distanceAccount = -0.3;
            }else {
                distanceAccount = 0;
            }

            if (RotateArm.getPosition() > ArmPositionMid) {
                double armExPos = (RotateArm.getPosition() - ArmPositionMid) * armRotateToArmExtendConstLEFT + (mainPivotVertOffSet*Mainpivottoextendconst) + ArmExtensionHome;
                ArmExtension.setPosition(armExPos + distanceAccount);
                secondRotate.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondRotateConstLEFT + secondRotateMiddle);
                setMainPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToMainConstLEFT + targetMainPivot + mainPivotVertOffSet);
                setSecondPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondPivotConstLEFT + (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst) + (mainPivotVertOffSet * Mainpivottosecondconst));

            } else {
                double armExPos = (RotateArm.getPosition() - ArmPositionMid) * armRotateToArmExtendConstRIGHT + (mainPivotVertOffSet*Mainpivottoextendconst) + ArmExtensionHome;
                ArmExtension.setPosition(armExPos + distanceAccount);
                secondRotate.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondRotateConstRIGHT + secondRotateMiddle);
                setMainPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToMainConstRIGHT + targetMainPivot + mainPivotVertOffSet);
                setSecondPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondPivotConstRIGHT + (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst) + (mainPivotVertOffSet * Mainpivottosecondconst));


            }

        }

        if (Objects.requireNonNull(armstateCurrent) == armState.delivery){

            double distanceAccount = 0;

            if (odometry.X > 280 && odometry.X < 300){
                distanceAccount = -0.32;
            } else if (odometry.X > 300 && odometry.X < 305){
                distanceAccount = -0.22;
            }else {
                distanceAccount = 0;
            }

            if (RotateArm.getPosition() > ArmPositionMid) {
                double armExPos = (RotateArm.getPosition() - ArmPositionMid) * armRotateToArmExtendConstLEFT + (mainPivotVertOffSet*Mainpivottoextendconst) + ArmExtensionHome;
                ArmExtension.setPosition(armExPos + distanceAccount);
                secondRotate.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondRotateConstLEFT + secondRotateMiddle);
                setMainPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToMainConstLEFT + targetMainPivot + mainPivotVertOffSet);
                setSecondPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondPivotConstLEFT + (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst) + (mainPivotVertOffSet * Mainpivottosecondconst));

            } else {
                double armExPos = (RotateArm.getPosition() - ArmPositionMid) * armRotateToArmExtendConstRIGHT + (mainPivotVertOffSet*Mainpivottoextendconst) + ArmExtensionHome;
                ArmExtension.setPosition(armExPos + distanceAccount);
                secondRotate.setPosition((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondRotateConstRIGHT + secondRotateMiddle);
                setMainPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToMainConstRIGHT + targetMainPivot + mainPivotVertOffSet);
                setSecondPivot((RotateArm.getPosition() - ArmPositionMid) * armRotateToSecondPivotConstRIGHT + (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst) + (mainPivotVertOffSet * Mainpivottosecondconst));


            }

        }

    }

    public void updateArm (double slidesPos, double adjustFactor){

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
            case delivery:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                armstateTarget = armState.moving;

                pivotMoveTimeDelivery.reset();

                DeliveryMoving = true;

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                double distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                RobotLog.d("distance reading 1: " + distance);

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 2: " + distance);
                }

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 3: " + distance);
                }

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 4: " + distance);
                }

                if(distance > 30 || distance < 4){
                    distance = sensors.backBoard.getDistance(DistanceUnit.CM);
                    RobotLog.d("distance reading 5: " + distance);
                }

                mainPivotOffSet = adjustFactor + ((distance - mindistancemm) * mainservopospermm);

                RobotLog.d("mainPivotOffSet" + mainPivotOffSet);

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                targetMainPivot = Range.clip(targetMainPivot, 0,1);

                timeToWaitDelivery = Math.max((Math.abs(getSecondPivotPosition() - targetMainPivot) * 180) * timePerDegree, (Math.abs(getTopPivotPosition() - (deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst)) * 180) * timePerDegree);

                RobotLog.d("targetMainPivot" + targetMainPivot);

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

                if (DeliveryMoving && pivotMoveTimeDelivery.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.delivery;
                    DeliveryMoving = false;
                }

                if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.deliverAuto;
                    DeliveryMovingAuto = false;
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                }

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

            if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                armstateCurrent = armState.collect;
                CollectionMoving = false;
            }
        }

    }

    public void updateArm (double slidesPos, boolean useDig){

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
            case delivery:

                secondRotate.setPosition(secondRotateMiddle);

                armstateCurrent = armState.moving;

                pivotMoveTimeDelivery.reset();

                setClaws(clawClosed);

                setGripperState(GripperState.closed);

                RotateClaw.setPosition(rotateDeliver);

                if (!sensors.armSensor.isPressed() && getMainPivotPosition() < 1){
                    mainPivotOffSet += 0.008;
                }else{
                    armstateCurrent = armState.delivery;
                    armstateTarget = armState.delivery;
                }

                targetMainPivot = deliveryTopPivot - slidesPos * servoPosPerTick + mainPivotOffSet;

                setMainPivot(targetMainPivot);

                targetMainPivot = Range.clip(targetMainPivot, 0, 1);

                setSecondPivot(deliverySecondPivot + (-slidesPos * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

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

                if (DeliveryMovingAuto && pivotMoveTimeAuto.milliseconds() >= timeToWaitDelivery) {
                    armstateCurrent = armState.deliverAuto;
                    armstateTarget = armState.delivery;
                    DeliveryMovingAuto = false;
                }

                if (CollectionMoving && pivotMoveTimeCollection.milliseconds() >= timeToWaitCollection) {
                    armstateCurrent = armState.collect;
                    CollectionMoving = false;
                }

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

        ArmExtension.setPosition(1);

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