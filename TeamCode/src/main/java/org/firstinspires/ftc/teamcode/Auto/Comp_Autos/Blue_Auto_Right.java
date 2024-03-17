package org.firstinspires.ftc.teamcode.Auto.Comp_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.odometry;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

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

@Autonomous(name = "Blue_Auto_Right", group = "Newest auto's")
public class Blue_Auto_Right extends LinearOpMode implements CycleMethods {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    Vector2D startPosition = new Vector2D(getRealCoords(90), getRealCoords(23));

    /**Left Prop position*/
    //purple pixel
    Vector2D DPS1F = startPosition;
    Vector2D DPC1F = new Vector2D(getRealCoords(97), getRealCoords(150));
    Vector2D DPCT1F = new Vector2D(getRealCoords(66), getRealCoords(83));
    Vector2D DPE1F = new Vector2D(getRealCoords(93), getRealCoords(51));

    //yellow pixel
    Vector2D DYS1F = DPE1F;
    Vector2D DYC1F = new Vector2D(getRealCoords(34), getRealCoords(28));
    Vector2D DYE1F = new Vector2D(getRealCoords(97), getRealCoords(32));

    Vector2D DYS2F = DYE1F;
    Vector2D DYC2F = new Vector2D(getRealCoords(151), getRealCoords(32));
    Vector2D DYE2F = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3F = DYE2F;
    Vector2D DYC3F = new Vector2D(getRealCoords(291), getRealCoords(30));
    Vector2D DYE3F = new Vector2D(getRealCoords(295), getRealCoords(94));

    /**Middle Prop position*/
    //purple pixel
    Vector2D DPS1S = startPosition;
    Vector2D DPC1S = new Vector2D(getRealCoords(104), getRealCoords(125));
    Vector2D DPCT1S = new Vector2D(getRealCoords(66), getRealCoords(83));
    Vector2D DPE1S = new Vector2D(getRealCoords(51), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1S = DPE1S;
    Vector2D DYC1S = new Vector2D(getRealCoords(34), getRealCoords(28));
    Vector2D DYE1S = new Vector2D(getRealCoords(97), getRealCoords(32));

    Vector2D DYS2S = DYE1S;
    Vector2D DYC2S = new Vector2D(getRealCoords(151), getRealCoords(32));
    Vector2D DYE2S = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3S = DYE2S;
    Vector2D DYC3S = new Vector2D(getRealCoords(291), getRealCoords(30));
    Vector2D DYE3S = new Vector2D(getRealCoords(295), getRealCoords(94));

    /**Right Prop position*/

    //purple pixel
    Vector2D DPS1T = startPosition;
    Vector2D DPC1T = new Vector2D(getRealCoords(104), getRealCoords(125));
    Vector2D DPCT1T = new Vector2D(getRealCoords(66), getRealCoords(83));
    Vector2D DPE1T = new Vector2D(getRealCoords(51), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1T = DPE1T;
    Vector2D DYC1T = new Vector2D(getRealCoords(34), getRealCoords(28));
    Vector2D DYE1T = new Vector2D(getRealCoords(97), getRealCoords(32));

    Vector2D DYS2T = DYE1T;
    Vector2D DYC2T = new Vector2D(getRealCoords(151), getRealCoords(32));
    Vector2D DYE2T = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3T = DYE2T;
    Vector2D DYC3T = new Vector2D(getRealCoords(291), getRealCoords(30));
    Vector2D DYE3T = new Vector2D(getRealCoords(295), getRealCoords(94));

    /**delivery and collection points*/

    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(60));
    Vector2D CC1F = new Vector2D(getRealCoords(265), getRealCoords(29));
    Vector2D CE1F = new Vector2D(getRealCoords(119), getRealCoords(32));

    Vector2D CS2F = CE1F;
    Vector2D CC2F = new Vector2D(getRealCoords(48), getRealCoords(33));
    Vector2D CE2F = new Vector2D(getRealCoords(38), getRealCoords(90));

    /**Action points*/

    //preload
    Vector2D extendSlides = new Vector2D(getRealCoords(224), getRealCoords(32));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(90), getRealCoords(70));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(90), getRealCoords(70));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(76), getRealCoords(94));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(91), getRealCoords(78));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(76), getRealCoords(94));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(91), getRealCoords(78));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(180), getRealCoords(32));

    //collection
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(183), getRealCoords(32));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(145), getRealCoords(32));
    Vector2D reverseIntake = new Vector2D(getRealCoords(72), getRealCoords(150));

    /**path objects*/
    GenMethods purple = new GenMethods();
    GenMethods yellow = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();

    /**error from target*/

    double collectionError = 5;

    double deliveryError = 5;

    double IntakeControlError = 10;

    double HeadingControlError = 10;

    /**setPoints*/

    int slidesPosWhitePixels = 1000;

    int slidesPosYellowPixel = 800;

    /**hardware objects*/
    Odometry odometry = new Odometry(startPosition.getX(), startPosition.getY(), 270);

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

    /**collection Methods*/
    public void collectPixels(){
        delivery.ArmExtension.setPosition(1);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        sleep(1000);

        collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
        collection.updateIntakeHeight();

        sleep(1000);

        collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
        collection.updateIntakeHeight();

        sleep(1000);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(400);

        collection.setState(Collection.intakePowerState.reversedHalf);
        collection.updateIntakeState();

    }

    public void collectOnePixel(){

        delivery.ArmExtension.setPosition(1);

        delivery.setRightGripperState(Delivery.rightGripperState.open);
        delivery.updateGrippers();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        sleep(500);

        collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
        collection.updateIntakeHeight();

        sleep(1000);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(400);

        collection.setState(Collection.intakePowerState.reversedHalf);
        collection.updateIntakeState();

    }

    public void delivery_and_collect_2() throws InterruptedException {

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);
        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Build.built;

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

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > -5 && !onlyOnce && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > -5 && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

            } else if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > -5) {

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

                if (auto == Blue_Auto_Right.Auto.two) {

                    dropWhitePixelsWait(Delivery.PixelsAuto.whiteBlue);

                    phase = Blue_Auto_Right.Phase.finished;

                } else {

                    dropWhitePixels(Delivery.PixelsAuto.whiteBlue);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Blue_Auto_Right.Phase.second2;

                }
            }

        }

        if (pathing){

            pathing = follower.followPathAuto(targetHeading, odometry, drive);

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

    public void delivery_and_collect_4() throws InterruptedException{

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);
        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Build.built;

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

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > -5 && !onlyOnce && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > -5 && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);


            } else if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > -5) {

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

                if (auto == Auto.four) {

                    dropWhitePixelsWait(Delivery.PixelsAuto.whiteBlue);

                    phase = Blue_Auto_Right.Phase.finished;

                } else {

                    dropWhitePixels(Delivery.PixelsAuto.whiteBlue);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Phase.third2;

                }

            }

        }

        if (pathing){

            pathing = follower.followPathAuto(targetHeading, odometry, drive);

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

    public void delivery_and_collect_6() throws InterruptedException{

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Build.built;

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

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > -5 && !onlyOnce && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > -5 && delivery.getLeftgripperstate() == Delivery.leftGripperState.closed && delivery.getRightgripperstate() == Delivery.rightGripperState.closed) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

            } else if (odometry.X > (extendSlidesDelivery.getX()+20) && odometry.getVerticalVelocity() > -5) {

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

                dropWhitePixelsWait(Delivery.PixelsAuto.whiteBlue);

                build = Build.notBuilt;

                delivering = false;

                phase = Phase.finished;

            }

        }

        if (pathing){

            pathing = follower.followPathAuto(targetHeading, odometry, drive);

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
        four,
        six
    }

    enum Preload{
        purple,
        yellow
    }

    enum Build{
        built,
        notBuilt
    }

    Blue_Auto_Right.Phase phase = Phase.preload;

    Blue_Auto_Right.Build build = Build.notBuilt;

    Blue_Auto_Right.Auto auto = Blue_Auto_Right.Auto.preload;

    Preload preload = Preload.purple;

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
                auto = Blue_Auto_Right.Auto.two;
            } else if (gamepad1.b) {
                auto = Blue_Auto_Right.Auto.four;
            } else if (gamepad1.y) {
                auto = Blue_Auto_Right.Auto.preload;
            }else if (gamepad1.x) {
                lockIn = true;
            }

        }

        waitForStart();

        if (propPos == 1){

            purple.fourPoints(DPS1F, DPC1F, DPCT1F, DPE1F, true);

            yellow.threePoints(DYS1F, DYC1F, DYE1F);
            yellow.threePoints(DYS2F, DYC2F, DYE2F);
            yellow.threePoints(DYS3F, DYC3F, DYE3F, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Right.Phase.finished)){

                switch (phase){
                    case preload:

                        switch (preload){
                            case purple:

                                odometry.update();

                                if (build == Build.notBuilt){
                                    follower.setPath(purple.followablePath, purple.pathingVelocity);
                                    pathing = true;
                                    build = Build.built;
                                    targetHeading = 270;
                                    deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();
                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){
                                        targetHeading = 180;
                                    }

                                }else if (Math.abs(DPE1F.getX() - odometry.X) < deliveryError && Math.abs(DPE1F.getY() - odometry.Y) < deliveryError && !pathing){

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    collectOnePixel();

                                    build = Build.notBuilt;

                                    preload = Preload.yellow;

                                }

                                break;

                            case yellow:

                                odometry.update();

                                if (build == Build.notBuilt){
                                    follower.setPath(yellow.followablePath, yellow.pathingVelocity);
                                    pathing = true;
                                    build = Build.built;
                                    targetHeading = 180;
                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(extendSlides.getX() - odometry.X) < IntakeControlError && Math.abs(extendSlides.getY() - odometry.Y) < IntakeControlError && deliverySlides.getCurrentposition() < 50){

                                        delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

                                    }

                                    if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError && !gotTwo) {

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.off);
                                        collection.updateIntakeState();

                                    }

                                }else if (Math.abs(DYE3F.getX() - odometry.X) < deliveryError && Math.abs(DYE3F.getY() - odometry.Y) < deliveryError && !pathing){

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
                                        collect.threePoints(CS2F, CC2F, CE2F, true);

                                        deliver.threePoints(CE2F, CC2F, CS2F);
                                        deliver.threePoints(CE1F, CC1F, CS1F, true);

                                        build = Build.notBuilt;

                                    }

                                }
                                break;
                            default:
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

            purple.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true);

            yellow.threePoints(DYS1S, DYC1S, DYE1S);
            yellow.threePoints(DYS2S, DYC2S, DYE2S);
            yellow.threePoints(DYS3S, DYC3S, DYE3S, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Right.Phase.finished)){

                switch (phase){

                    case preload:

                        switch (preload){
                            case purple:

                                odometry.update();

                                if (build == Build.notBuilt){
                                    follower.setPath(purple.followablePath, purple.pathingVelocity);
                                    pathing = true;
                                    build = Build.built;
                                    targetHeading = 270;
                                    deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();
                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < HeadingControlError){
                                        targetHeading = 235;
                                    }

                                }else if (Math.abs(DPE1S.getX() - odometry.X) < collectionError && Math.abs(DPE1S.getY() - odometry.Y) < collectionError && !pathing){

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    collectOnePixel();

                                    build = Build.notBuilt;

                                    preload = Preload.yellow;

                                }

                                break;
                            case yellow:

                                odometry.update();

                                if (build == Build.notBuilt){

                                    follower.setPath(purple.followablePath, purple.pathingVelocity);

                                    pathing = true;

                                    build = Build.built;

                                    targetHeading = 270;

                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(oneEightyHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingS.getX() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        targetHeading = 180;

                                        delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

                                    }

                                    if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError && !gotTwo) {

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.off);
                                        collection.updateIntakeState();

                                    }

                                }else if (Math.abs(DYE3S.getX() - odometry.X) < deliveryError && Math.abs(DYE3S.getY() - odometry.Y) < deliveryError && !pathing){

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
                                        collect.threePoints(CS2F, CC2F, CE2F, true);

                                        deliver.threePoints(CE2F, CC2F, CS2F);
                                        deliver.threePoints(CE1F, CC1F, CS1F, true);

                                        build = Build.notBuilt;

                                    }

                                }

                                break;
                            default:
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

            purple.fourPoints(DPS1T, DPC1T, DPCT1T, DPE1T, true);

            yellow.threePoints(DYS1T, DYC1T, DYE1T);
            yellow.threePoints(DYS2T, DYC2T, DYE2T);
            yellow.threePoints(DYS3T, DYC3T, DYE3T, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Right.Phase.finished)){

                switch (phase){
                    case preload:

                        switch (preload){
                            case purple:

                                odometry.update();

                                if (build == Build.notBuilt){
                                    follower.setPath(purple.followablePath, purple.pathingVelocity);
                                    pathing = true;
                                    build = Build.built;
                                    targetHeading = 270;

                                    deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();

                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError){
                                        targetHeading = 235;
                                    }

                                }else if (Math.abs(DPE1T.getX() - odometry.X) < collectionError && Math.abs(DPE1T.getY() - odometry.Y) < collectionError && !pathing){

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    collectOnePixel();

                                    build = Build.notBuilt;

                                    preload = Preload.yellow;

                                }

                                break;
                            case yellow:

                                odometry.update();

                                if (build == Build.notBuilt){

                                    follower.setPath(purple.followablePath, purple.pathingVelocity);

                                    pathing = true;

                                    build = Build.built;

                                    targetHeading = 270;

                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingT.getX() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        targetHeading = 180;

                                        delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.whiteBlue, odometry);

                                    }

                                    if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError && !gotTwo) {

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.off);
                                        collection.updateIntakeState();

                                    }

                                }else if (Math.abs(DYE3T.getX() - odometry.X) < deliveryError && Math.abs(DYE3T.getY() - odometry.Y) < deliveryError && !pathing){

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
                                        collect.threePoints(CS2F, CC2F, CE2F, true);

                                        deliver.threePoints(CE2F, CC2F, CS2F);
                                        deliver.threePoints(CE1F, CC1F, CS1F, true);

                                        build = Build.notBuilt;

                                    }

                                }

                                break;
                            default:
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
