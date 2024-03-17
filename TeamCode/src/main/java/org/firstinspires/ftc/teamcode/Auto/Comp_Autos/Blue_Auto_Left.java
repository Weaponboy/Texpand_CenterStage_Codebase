package org.firstinspires.ftc.teamcode.Auto.Comp_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

@Autonomous(name = "Blue_Auto_Left", group = "Newest auto's")
public class Blue_Auto_Left extends LinearOpMode implements CycleMethods {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**
     * drop preload pixels
     * */
    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1F = new Vector2D(getRealCoords(241), getRealCoords(42));
    Vector2D DPCT1F = new Vector2D(getRealCoords(188), getRealCoords(78));
    Vector2D DPE1F = new Vector2D(getRealCoords(305), getRealCoords(75));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(76));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(65));
    Vector2D DPE1S = new Vector2D(getRealCoords(305), getRealCoords(95));

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(223), getRealCoords(79));
    Vector2D DPCT1T = new Vector2D(getRealCoords(164), getRealCoords(110));
    Vector2D DPE1T = new Vector2D(getRealCoords(305), getRealCoords(105));

    Vector2D DS1F = new Vector2D(getRealCoords(312), getRealCoords(143));

    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(292), getRealCoords(154));
    Vector2D CE1F = new Vector2D(getRealCoords(240), getRealCoords(150));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(46), getRealCoords(150));

    Vector2D lastPoint = new Vector2D(getRealCoords(38), getRealCoords(150));

    /**Action points*/
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(180), getRealCoords(150));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(102), getRealCoords(150));
    Vector2D reverseIntake = new Vector2D(getRealCoords(72), getRealCoords(150));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(260), getRealCoords(90));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(220), getRealCoords(65));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(241), getRealCoords(86));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(218), getRealCoords(78));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(227), getRealCoords(100));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(210), getRealCoords(50));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(160), getRealCoords(180));

    GenMethods preloadPaths = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods deliverLast = new GenMethods();

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 23, 270);

    Drivetrain drive = new Drivetrain();

    mecanumFollower follower = new mecanumFollower();

    ElapsedTime gripperControl = new ElapsedTime();

    /**booleans*/
    boolean lockIn = false;

    boolean pathing = false;

    boolean delivering = false;

    boolean gotTwo = false;

    boolean onlyOnce = true;

    /**doubles*/
    double targetHeading = 0;

    double timeChanger;

    /**error from target*/

    double collectionError = 5;

    double deliveryError = 5;

    double IntakeControlError = 10;

    double HeadingControlError = 10;

    /**setPoints*/

    double armPosLeft = 0.5;

    double armPosMiddle = 0.55;

    double armPosRight = 0.35;

    double armPosWhitePixels = 0.46;

    int slidesPosWhitePixels = 1000;

    int slidesPosYellowPixel = 800;

    /**enums*/

    enum Phase{
        preload,
        first2,
        second2,
        third2,
        finished
    }

    enum Auto{
        preload,
        two,
        four
    }

    enum Build{
        built,
        notBuilt
    }

    Blue_Auto_Left.Phase phase = Phase.preload;

    Blue_Auto_Left.Build build = Build.notBuilt;

    Blue_Auto_Left.Auto auto = Blue_Auto_Left.Auto.preload;

    /**collection Methods*/
    public void collectPixels(){

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        collection.setIntakeHeight(Collection.intakeHeightState.collect);
        collection.updateIntakeHeight();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        sleep(2000);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(200);

        collection.setState(Collection.intakePowerState.reversedHalf);
        collection.updateIntakeState();

        collection.setIntakeHeight(Collection.intakeHeightState.hangStowed);
        collection.updateIntakeHeight();

        sleep(200);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

//
//        boolean gotTwo;
//
//        delivery.ArmExtension.setPosition(0.96);
//
//        delivery.setGripperState(Delivery.GripperState.open);
//        delivery.updateGrippers();
//
//        collection.setState(Collection.intakePowerState.on);
//        collection.updateIntakeState();
//
//        sleep(500);
//
//        collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
//        collection.updateIntakeHeight();
//
//        gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
//
//        if (gotTwo){
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//        }else {
//            sleep(1000);
//
//            collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
//            collection.updateIntakeHeight();
//
//            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
//
//            if (gotTwo){
//                delivery.setGripperState(Delivery.GripperState.closed);
//                delivery.updateGrippers();
//            }else {
//                sleep(1000);
//
//                gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
//
//                if (gotTwo){
//                    delivery.setGripperState(Delivery.GripperState.closed);
//                    delivery.updateGrippers();
//                }else {
//                    sleep(1000);
//
//                    delivery.setGripperState(Delivery.GripperState.closed);
//                    delivery.updateGrippers();
//                }
//            }
//
//        }
//
//
//        sleep(200);
//
//        collection.setState(Collection.intakePowerState.reversedHalf);
//        collection.updateIntakeState();
//
//        collection.setIntakeHeight(Collection.intakeHeightState.hangStowed);
//        collection.updateIntakeHeight();
//
//        sleep(200);
//
//        collection.setState(Collection.intakePowerState.off);
//        collection.updateIntakeState();
    }

    public void delivery_and_collect_2() throws InterruptedException {

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);
        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Blue_Auto_Left.Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Blue_Auto_Left.Build.built;

            targetHeading = 180;

            delivering = false;

            delivery.setGripperState(Delivery.GripperState.open);

            delivery.updateGrippers();

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);

            collection.updateIntakeHeight();

        }
//
//        if (!delivering){
//
//            if (Math.abs(turnIntakeOn.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOn.getY() - odometry.Y) < IntakeControlError){
//
//                delivery.setGripperState(Delivery.GripperState.open);
//                delivery.updateGrippers();
//
//                collection.setIntakeHeight(Collection.intakeHeightState.hangStowed);
//                collection.updateIntakeHeight();
//            }
//
//        }
//
//        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
//            delivery.setRightGripperState(Delivery.rightGripperState.closed);
//            delivery.updateGrippers();
//        }
//
//        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
//            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
//            delivery.updateGrippers();
//        }
//
//        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){
//
//            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 150));
//
//            if (lengthToEnd > 30){
//                timeChanger = 0;
//            } else if (lengthToEnd < 30) {
//                timeChanger = 400;
//            }
//
//            gripperControl.reset();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//            drive.RF.setPower(0);
//            drive.RB.setPower(0);
//            drive.LF.setPower(0);
//            drive.LB.setPower(0);
//
//            follower.setPath(deliver.followablePath, deliver.pathingVelocity);
//
//            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
//
//            delivering = true;
//
//            onlyOnce = false;
//
//            gotTwo = true;
//
//        }
//
//        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+500) && gotTwo){
//
//            collection.setState(Collection.intakePowerState.reversed);
//            collection.updateIntakeState();
//
//        }
//
//        if (gripperControl.milliseconds() > (timeChanger+500) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo && odometry.X < 170){
//
//            collection.setState(Collection.intakePowerState.on);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.open);
//            delivery.updateGrippers();
//
//        } else if (gripperControl.milliseconds() > (timeChanger+500) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo && odometry.X > 170) {
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//        }
//
//        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1100) && gotTwo){
//
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && !onlyOnce && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                targetHeading = 135;

            } else if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > 5 && !onlyOnce) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > 5 && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

            } else if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > 5) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

//            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError && !gotTwo) {
//
//                delivery.setGripperState(Delivery.GripperState.closed);
//                delivery.updateGrippers();
//
//                collection.setState(Collection.intakePowerState.off);
//                collection.updateIntakeState();
//
//            }
//
//            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < IntakeControlError && !gotTwo){
//
//                delivery.setGripperState(Delivery.GripperState.closed);
//                delivery.updateGrippers();
//
//                collection.setState(Collection.intakePowerState.reversedHalf);
//                collection.updateIntakeState();
//
//            }

            if (Math.abs(DS1F.getX() - odometry.X) < 4 && Math.abs(DS1F.getY() - odometry.Y) < 4 && !pathing) {

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                if (auto == Blue_Auto_Left.Auto.two) {

                    dropWhitePixelsWait(Delivery.PixelsAuto.whiteBlue);

                    phase = Blue_Auto_Left.Phase.finished;

                } else {

                    dropWhitePixels(Delivery.PixelsAuto.whiteBlue);

                    build = Blue_Auto_Left.Build.notBuilt;

                    delivering = false;

                    phase = Blue_Auto_Left.Phase.second2;

                }
            }
        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 3);
            }


        }else if (Math.abs(CE2F.getX() - odometry.X) < collectionError && Math.abs(CE2F.getY() - odometry.Y) < collectionError - 2){

            if (!gotTwo){

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                collectPixels();

            }else {
                gotTwo = false;
            }

            pathing = true;

            onlyOnce = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

        }
    }

    public void delivery_and_collect_4() throws InterruptedException{

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);
        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Blue_Auto_Left.Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Blue_Auto_Left.Build.built;

            targetHeading = 180;

            delivering = false;

            delivery.setGripperState(Delivery.GripperState.open);

            delivery.updateGrippers();

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);

            collection.updateIntakeHeight();

        }

//        if (!delivering){
//
//            if (Math.abs(turnIntakeOn.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOn.getY() - odometry.Y) < IntakeControlError){
//
//                delivery.setGripperState(Delivery.GripperState.open);
//                delivery.updateGrippers();
//
//                collection.setIntakeHeight(Collection.intakeHeightState.hangStowed);
//                collection.updateIntakeHeight();
//            }
//
//        }
//
//        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
//            delivery.setRightGripperState(Delivery.rightGripperState.closed);
//            delivery.updateGrippers();
//        }
//
//        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
//            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
//            delivery.updateGrippers();
//        }
//
//        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){
//
//            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 150));
//
//            if (lengthToEnd > 30){
//                timeChanger = 0;
//            } else if (lengthToEnd < 30) {
//                timeChanger = 400;
//            }
//
//            gripperControl.reset();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//            drive.RF.setPower(0);
//            drive.RB.setPower(0);
//            drive.LF.setPower(0);
//            drive.LB.setPower(0);
//
//            follower.setPath(deliver.followablePath, deliver.pathingVelocity);
//
//            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
//
//            delivering = true;
//
//            onlyOnce = false;
//
//            gotTwo = true;
//
//        }
//
//        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+500) && gotTwo){
//
//            collection.setState(Collection.intakePowerState.reversed);
//            collection.updateIntakeState();
//
//        }
//
//        if (gripperControl.milliseconds() > (timeChanger+500) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo && odometry.X < 170){
//
//            collection.setState(Collection.intakePowerState.on);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.open);
//            delivery.updateGrippers();
//
//        } else if (gripperControl.milliseconds() > (timeChanger+500) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo && odometry.X > 170) {
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//        }
//
//        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1100) && gotTwo){
//
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && !onlyOnce && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                targetHeading = 135;

            } else if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > 5 && !onlyOnce) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > 5 && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

            } else if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > 5) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

//            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError && !gotTwo) {
//
//                delivery.setGripperState(Delivery.GripperState.closed);
//                delivery.updateGrippers();
//
//                collection.setState(Collection.intakePowerState.off);
//                collection.updateIntakeState();
//
//            }

//            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < IntakeControlError && !gotTwo){
//
//                delivery.setGripperState(Delivery.GripperState.closed);
//                delivery.updateGrippers();
//
//                collection.setState(Collection.intakePowerState.reversedHalf);
//                collection.updateIntakeState();
//
//            }

            if (Math.abs(DS1F.getX() - odometry.X) < 4 && Math.abs(DS1F.getY() - odometry.Y) < 4 && !pathing) {

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                if (auto == Auto.four) {

                    dropWhitePixelsWait(Delivery.PixelsAuto.whiteBlue);

                    phase = Blue_Auto_Left.Phase.finished;

                } else {

                    dropWhitePixels(Delivery.PixelsAuto.whiteBlue);

                    build = Blue_Auto_Left.Build.notBuilt;

                    delivering = false;

                    phase = Blue_Auto_Left.Phase.second2;

                }
            }
        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 3);
            }


        }else if (Math.abs(CE2F.getX() - odometry.X) < collectionError && Math.abs(CE2F.getY() - odometry.Y) < collectionError-2){

            if (!gotTwo){

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                collectPixels();

            }else {
                gotTwo = false;
            }

            pathing = true;

            onlyOnce = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

        }
    }

    public void delivery_and_collect_6() throws InterruptedException{

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Blue_Auto_Left.Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Blue_Auto_Left.Build.built;

            targetHeading = 180;

            delivering = false;

            delivery.setGripperState(Delivery.GripperState.open);

            delivery.updateGrippers();

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);

            collection.updateIntakeHeight();

        }

        if (!delivering){

            if (Math.abs(turnIntakeOn.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOn.getY() - odometry.Y) < IntakeControlError){
                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();
            }

        }

        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
            delivery.setRightGripperState(Delivery.rightGripperState.closed);
            delivery.updateGrippers();
        }

        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
            delivery.updateGrippers();
        }

        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

            if (lengthToEnd > 30){
                timeChanger = 0;
            } else if (lengthToEnd < 30) {
                timeChanger = 400;
            }

            gripperControl.reset();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            delivering = true;

            onlyOnce = false;

            gotTwo = true;

        }

        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+500) && gotTwo){

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+500) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo && odometry.X < 170){

            collection.setState(Collection.intakePowerState.on);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.open);
            delivery.updateGrippers();

        } else if (gripperControl.milliseconds() > (timeChanger+500) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo && odometry.X > 170) {
            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();
        }

        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1100) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > 5 && !onlyOnce && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > 5 && !onlyOnce) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > 5 && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);


            } else if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > 5) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError && !gotTwo) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < IntakeControlError && !gotTwo){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversedHalf);
                collection.updateIntakeState();

            }

            if (Math.abs(CS1F.getX() - odometry.X) < deliveryError && Math.abs(CS1F.getY() - odometry.Y) < deliveryError) {

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

                build = Blue_Auto_Left.Build.notBuilt;

                delivering = false;

                phase = Blue_Auto_Left.Phase.finished;

            }

        }

        if (pathing){

            pathing = follower.followPathAuto(targetHeading, odometry, drive);

        }else if (Math.abs(CE2F.getX() - odometry.X) < collectionError && Math.abs(CE2F.getY() - odometry.Y) < collectionError){

            deliverLast.twoPoints(new Vector2D(odometry.X, odometry.Y), lastPoint, true);

            follower.setPath(deliverLast.followablePath, deliverLast.pathingVelocity);

            pathing = true;

        }else if (Math.abs(CE2F.getX() - odometry.X) < collectionError && Math.abs(CE2F.getY() - odometry.Y) < collectionError){

            if (!gotTwo){

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                collectPixels();

            }else {
                gotTwo = false;
            }

            pathing = true;

            onlyOnce = false;

            delivering = true;

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

        }

    }

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        while(!lockIn){

            telemetry.addData("Auto activated", auto);
            telemetry.addData("press y for 2+0", "");
            telemetry.addData("press a for 2+2", "");
            telemetry.addData("press b for 2+4", "");
            telemetry.addData("press x to lock in!!!!", "");
            telemetry.update();

            if (gamepad1.a){
                auto = Blue_Auto_Left.Auto.two;
            } else if (gamepad1.b) {
                auto = Blue_Auto_Left.Auto.four;
            } else if (gamepad1.y) {
                auto = Blue_Auto_Left.Auto.preload;
            }else if (gamepad1.x) {
                lockIn = true;
            }

        }

        waitForStart();

        propPos = 2;

        if (propPos == 1){

            preloadPaths.fourPoints(DPS1F, DPC1F, DPCT1F, DPE1F, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Left.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && !(targetHeading == 320)){
                                targetHeading = 290;
                            }

                            if (Math.abs(oneEightyHeadingF.getX() - odometry.X) < HeadingControlError+10 && Math.abs(oneEightyHeadingF.getY() - odometry.Y) < HeadingControlError+10 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);


                            }


                        }else if (Math.abs(DPE1F.getX() - odometry.X) < deliveryError && Math.abs(DPE1F.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){
                                dropYellowPixelWait(Delivery.PixelsAuto.yellow1Blue, odometry);
                                phase = Phase.finished;
                            }else {

                                dropYellowPixel(Delivery.PixelsAuto.yellow1Blue, odometry);

                                phase = Phase.first2;

                                collect.threePoints(CS1F, CC1F, CE1F);
                                collect.twoPoints(CS2F, CE2F, true);

                                deliver.twoPoints(CE2F, CS2F);
                                deliver.threePoints(CE1F, CC1F, DS1F, true);

                                build = Build.notBuilt;

                            }

                        }

                        break;
                    case first2:
                        delivery_and_collect_2();
                        break;
                    case second2:
                        delivery_and_collect_4();
                        break;
                    case third2:
                        delivery_and_collect_6();
                        break;
                    default:
                }
            }

        } else if (propPos == 2) {

            preloadPaths.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Left.Phase.finished) && opModeIsActive()){

                switch (phase){
                    case preload:

                        odometry.update();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive, 3);

                            if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < 10 && !(targetHeading == 310)){
                                targetHeading = 310;
                            }

                            if (Math.abs(oneEightyHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingS.getY() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

                            }


                        }else if (Math.abs(DPE1S.getX() - odometry.X) < deliveryError && Math.abs(DPE1S.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){

                                dropYellowPixelWait(Delivery.PixelsAuto.yellow2Blue, odometry);

                                phase = Phase.finished;

                            }else {

                                dropYellowPixel(Delivery.PixelsAuto.yellow2Blue, odometry);

                                phase = Phase.first2;

                                collect.threePoints(CS1F, CC1F, CE1F);
                                collect.twoPoints(CS2F, CE2F, true);

                                deliver.twoPoints(CE2F, CS2F);
                                deliver.threePoints(CE1F, CC1F, DS1F, true);

                                build = Build.notBuilt;

                            }

                        }

                        break;
                    case first2:
                        delivery_and_collect_2();
                        break;
                    case second2:
                        delivery_and_collect_4();
                        break;
                    case third2:
                        delivery_and_collect_6();
                        break;
                    default:
                }
            }

        }else if (propPos == 3) {

            preloadPaths.fourPoints(DPS1T, DPC1T, DPCT1T, DPE1T, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Left.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                targetHeading = 350;
                            }

                            if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingT.getY() - odometry.Y) < HeadingControlError){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

                            }

                        }else if (Math.abs(DPE1T.getX() - odometry.X) < deliveryError && Math.abs(DPE1T.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){

                                dropYellowPixelWait(Delivery.PixelsAuto.yellow3Blue, odometry);
                                phase = Phase.finished;

                            }else {

                                dropYellowPixel(Delivery.PixelsAuto.yellow3Blue, odometry);

                                phase = Phase.first2;

                                collect.threePoints(CS1F, CC1F, CE1F);
                                collect.twoPoints(CS2F, CE2F, true);

                                deliver.twoPoints(CE2F, CS2F);
                                deliver.threePoints(CE1F, CC1F, DS1F, true);

                                build = Build.notBuilt;

                            }

                        }

                        break;
                    case first2:
                        delivery_and_collect_2();
                        break;
                    case second2:
                        delivery_and_collect_4();
                        break;
                    case third2:
                        delivery_and_collect_6();
                        break;
                    default:
                }

            }

        }

    }

    public void initialize(){

        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        init(hardwareMap);

        sensors.initAprilTag(telemetry, false);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

    }

}
