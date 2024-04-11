package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue.Far;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue.Close.Blue_Close_Stage;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Red.Far.Red_Far_Stage;
import org.firstinspires.ftc.teamcode.Auto.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@Autonomous(name = "Blue_Far_Stage", group = "blue auto's")
public class Blue_Far_Stage extends LinearOpMode implements CycleMethods {

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
    Vector2D DPC1F = new Vector2D(getRealCoords(85), getRealCoords(167));
    Vector2D DPE1F = new Vector2D(getRealCoords(36), getRealCoords(150));

    Vector2D DYS2F = DPE1F;
    Vector2D DYE2F = new Vector2D(getRealCoords(240), getRealCoords(154));

    Vector2D DYS3F = DYE2F;
    Vector2D DYC3F = new Vector2D(getRealCoords(260), getRealCoords(154));
    Vector2D DYE3F = new Vector2D(getRealCoords(310), getRealCoords(84));

    /**delivery and collection points*/
    //segment 1 deliver straight
    Vector2D DS1F = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1F = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1F = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 2
    Vector2D DS2F = DE1F;
    Vector2D DE2F = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3F = DE2F;
    Vector2D DC3F = new Vector2D(getRealCoords(220), getRealCoords(137));
    Vector2D DE3F = new Vector2D(getRealCoords(320), getRealCoords(120));

    /**collecting paths*/

    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(312), getRealCoords(162));
    Vector2D CE1F = new Vector2D(getRealCoords(220), getRealCoords(156));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3F = CE2F;
    Vector2D CE3F = new Vector2D(getRealCoords(35.5), getRealCoords(150));

    Vector2D CS3FS = CE3F;
    Vector2D CC3FS = new Vector2D(getRealCoords(72), getRealCoords(153));
    Vector2D CE3FS = new Vector2D(getRealCoords(36), getRealCoords(120));

    /**Middle Prop position*/
    //purple pixel
    Vector2D DPS1S = startPosition;
    Vector2D DPC1S = new Vector2D(getRealCoords(70), getRealCoords(166));
    Vector2D DPE1S = new Vector2D(getRealCoords(44), getRealCoords(152));

    Vector2D DYS2S = DPE1S;
    Vector2D DYE2S = new Vector2D(getRealCoords(240), getRealCoords(154));

    Vector2D DYS3S = DYE2S;
    Vector2D DYC3S = new Vector2D(getRealCoords(275), getRealCoords(158));
    Vector2D DYE3S = new Vector2D(getRealCoords(310), getRealCoords(97));

    /**delivery and collection points*/
    //segment 1 deliver straight
    Vector2D DS1S = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1S = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1S = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 2
    Vector2D DS2S = DE1S;
    Vector2D DE2S = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3S = DE2S;
    Vector2D DC3S = new Vector2D(getRealCoords(220), getRealCoords(137));
    Vector2D DE3S = new Vector2D(getRealCoords(320), getRealCoords(120));

    /**collecting paths*/

    Vector2D CS1S = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1S = new Vector2D(getRealCoords(312), getRealCoords(162));
    Vector2D CE1S = new Vector2D(getRealCoords(220), getRealCoords(156));

    Vector2D CS2S = CE1S;
    Vector2D CE2S = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3S = CE2S;
    Vector2D CE3S = new Vector2D(getRealCoords(35.5), getRealCoords(150));

    Vector2D CS3SS = CE3S;
    Vector2D CC3SS = new Vector2D(getRealCoords(72), getRealCoords(153));
    Vector2D CE3SS = new Vector2D(getRealCoords(36), getRealCoords(120));

    /**Right Prop position*/
    //purple pixel
    Vector2D DPS1T = startPosition;
    Vector2D DPE1T = new Vector2D(getRealCoords(74), getRealCoords(74));

    Vector2D DPS2T = DPE1T;
    Vector2D DPC2T = new Vector2D(getRealCoords(98), getRealCoords(65));
    Vector2D DPE2T = new Vector2D(getRealCoords(89), getRealCoords(150));

    Vector2D DYS1T = DPE1T;
    Vector2D DYE1T = new Vector2D(getRealCoords(44), getRealCoords(152));

    Vector2D DYS2T = DYE1T;
    Vector2D DYE2T = new Vector2D(getRealCoords(240), getRealCoords(154));

    Vector2D DYS3T = DYE2T;
    Vector2D DYC3T = new Vector2D(getRealCoords(265), getRealCoords(154));
    Vector2D DYE3T = new Vector2D(getRealCoords(310), getRealCoords(106));

    /**delivery and collection points*/
    //segment 1 deliver straight
    Vector2D DS1T = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1T = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1T = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 2
    Vector2D DS2T = DE1T;
    Vector2D DE2T = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3T = DE2T;
    Vector2D DC3T = new Vector2D(getRealCoords(220), getRealCoords(137));
    Vector2D DE3T = new Vector2D(getRealCoords(320), getRealCoords(120));

    /**collecting paths*/

    Vector2D CS1T = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1T = new Vector2D(getRealCoords(312), getRealCoords(162));
    Vector2D CE1T = new Vector2D(getRealCoords(220), getRealCoords(156));

    Vector2D CS2T = CE1T;
    Vector2D CE2T = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3T = CE2T;
    Vector2D CE3T = new Vector2D(getRealCoords(35.5), getRealCoords(150));

    Vector2D CS3TS = CE3T;
    Vector2D CC3TS = new Vector2D(getRealCoords(72), getRealCoords(153));
    Vector2D CE3TS = new Vector2D(getRealCoords(36), getRealCoords(120));

    /**Action points*/

    //first position
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(90), getRealCoords(40));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(77), getRealCoords(91));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(82), getRealCoords(70));

    //third position
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(85), getRealCoords(38));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(150), getRealCoords(180));
    Vector2D armOverPosition = new Vector2D(getRealCoords(180), getRealCoords(180));

    //collection
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(180), getRealCoords(150));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(125), getRealCoords(150));
    Vector2D reverseIntake = new Vector2D(getRealCoords(45), getRealCoords(150));
    Vector2D restartIntake = new Vector2D(getRealCoords(65), getRealCoords(150));

    Vector2D lastStack = new Vector2D();

    /**path objects*/
    GenMethods purple = new GenMethods();
    GenMethods whitePixelSeg1 = new GenMethods();
    GenMethods whitePixelSeg2 = new GenMethods();
    GenMethods yellow = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods collectSecond = new GenMethods();

    /**error from target*/

    double collectionError = 5;

    double deliveryError = 5;

    double IntakeControlError = 10;

    double HeadingControlError = 10;

    /**setPoints*/

    int slidesPosWhitePixels = 1000;

    int slidesPosYellowPixel = 400;

    /**hardware objects*/
    Odometry odometry = new Odometry(startPosition.getX(), startPosition.getY(), 270);

    Drivetrain drive = new Drivetrain();

    mecanumFollower follower = new mecanumFollower();

    ElapsedTime gripperControl = new ElapsedTime();

    public void clearAll(){
        yellow.ClearAll();
        purple.ClearAll();
        whitePixelSeg1.ClearAll();
        whitePixelSeg2.ClearAll();
        deliver.ClearAll();
        collect.ClearAll();
        collectSecond.ClearAll();
    }

    /**booleans*/
    boolean lockIn = false;

    boolean pathing = false;

    boolean delivering = false;

    boolean gotTwo = false;

    boolean onlyOnce = true;

    boolean sensorTouched = false;

    /**doubles*/
    double targetHeading = 0;

    double timeChanger;

    boolean armOver = false;

    double intakeCurrentOne = 5500; 

    double intakeNormal = 5500;

    boolean reversingIntake = false;
    Collection.intakePowerState previousState = Collection.intakePowerState.off;
    ElapsedTime reverseIntakeTimer = new ElapsedTime();
    ElapsedTime autoTimer = new ElapsedTime();

    Vector2D DeliveryEndpoint;
    Vector2D CollectionEndpoint;

    /**collection Methods*/
    public void collectOnePixel(){

        int counter;

        delivery.setRightGripperState(Delivery.rightGripperState.open);
        delivery.updateGrippers();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        sleep(200);

        collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
        collection.updateIntakeHeight();

        counter = 0;

        while (counter < 20){

            counter++;
            sleep(50);
            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

            if (collection.getIntakeCurrentUse() > intakeCurrentOne && !reversingIntake){
                reversingIntake = true;
                reverseIntakeTimer.reset();
                previousState = collection.getPowerState();
                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();
            }

            if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
                collection.setState(previousState);
                collection.updateIntakeState();
                reversingIntake = false;
            }

            if (gotTwo){
                counter = 45;
            }

        }

        while (counter < 40){

            counter++;

            sleep(50);

            collection.IntakeHeightRight.setPosition(collection.getIntakeHeightRight() - 0.005);

            if (collection.getIntakeCurrentUse() > 6000 && !reversingIntake){
                reversingIntake = true;
                reverseIntakeTimer.reset();
                previousState = collection.getPowerState();
                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();
            }

            if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
                collection.setState(previousState);
                collection.updateIntakeState();
                reversingIntake = false;
            }

            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

            if (gotTwo){
                counter = 45;
            }

        }

    }

    public void delivery_and_collect_2() throws InterruptedException {

        odometry.update();
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

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

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setIntakeHeight(Collection.intakeHeightState.fifthAndHalf);
                collection.updateIntakeHeight();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

                delivery.ArmExtension.setPosition(delivery.ArmExtensionHome);

            }

            if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                gripperControl.reset();

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

                delivering = true;

                armOver = false;

                gotTwo = true;

            }

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX()) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPosition.getX()){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                armOver = true;

            }

            if (sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200){

                drive.setAllPower(0.4);

                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                pathing = true;

                gotTwo = false;

                armOver = false;

                sensorTouched = true;

                if (auto == Auto.two) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    deliverySlides.DeliverySlides(0, -0.2);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.setArmTargetState(Delivery.armState.collect);
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    deliverySlides.DeliverySlides(0, -0.5);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Phase.second2;

                }

            }

//            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 10 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){
//
//                drive.setAllPower(0.4);
//
//                delivery.setGripperState(Delivery.GripperState.open);
//
//                delivery.updateGrippers();
//
//                sleep(200);
//
//                delivery.setArmTargetState(Delivery.armState.collect);
//                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
//
//                pathing = true;
//
//                gotTwo = false;
//
//                armOver = false;
//
//                if (auto == Auto.two) {
//
//                    phase = Phase.finished;
//
//                    drive.setAllPower(0);
//
//                    deliverySlides.DeliverySlides(0, -0.2);
//
//                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
//                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
//                    }
//
//                } else {
//
//                    deliverySlides.DeliverySlides(0, -0.5);
//
//                    build = Build.notBuilt;
//
//                    delivering = false;
//
//                    phase = Phase.second2;
//
//                }
//
//            }

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();

            }

            if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (Math.abs(DeliveryEndpoint.getX() - odometry.X) < 4 && Math.abs(DeliveryEndpoint.getY() - odometry.Y) < 4 && !pathing) {

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.two) {

                    phase = Phase.finished;

                    deliverySlides.DeliverySlides(0, -0.2);

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    deliverySlides.DeliverySlides(0, -0.5);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Phase.second2;

                }
            }

        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2, 20);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 1.5, 25);
            }

        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError - 2){

            if(auto == Auto.two){
                collectStraight(autoTimer, drive, Collection.intakeHeightState.fifthPixel, Collection.intakeHeightState.forthPixel, 2);
            } else if (auto == Auto.four) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.fifthPixel, Collection.intakeHeightState.forthPixel, 1.5);
            }else if (auto == Auto.six) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.fifthPixel, Collection.intakeHeightState.forthPixel, 0.8);
            }
            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
        }

    }

    public void delivery_and_collect_4() throws InterruptedException{

        odometry.update();
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

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

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                collection.updateIntakeHeight();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

                delivery.ArmExtension.setPosition(delivery.ArmExtensionHome);

            }

            if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                gripperControl.reset();

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

                delivering = true;

                armOver = false;

                gotTwo = true;

            }

        }

        if (collection.getIntakeCurrentUse() > intakeNormal && !reversingIntake){
            reversingIntake = true;
            reverseIntakeTimer.reset();
            previousState = collection.getPowerState();
            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();
        }

        if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
            collection.setState(previousState);
            collection.updateIntakeState();
            reversingIntake = false;
        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX()) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPosition.getX()){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                armOver = true;

            }

            if (sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200){

                drive.setAllPower(0.4);

                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                sensorTouched = true;

                if (auto == Auto.four){
                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }
                }else{
                    phase = Phase.third2;
                    build = Build.notBuilt;
                    delivering = false;
                }

            }

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 5 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){

                drive.setAllPower(0.4);
                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.four){
                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }
                }else{
                    phase = Phase.third2;
                    build = Build.notBuilt;
                    delivering = false;
                }

            }

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();

            }

            if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

            }

            if (Math.abs(DeliveryEndpoint.getX() - odometry.X) < 4 && Math.abs(DeliveryEndpoint.getY() - odometry.Y) < 4 && !pathing) {

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -1);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.four){
                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }
                }else{
                    phase = Phase.third2;
                    build = Build.notBuilt;
                    delivering = false;
                }

            }

        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2, 20);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 1.5, 25);
            }


        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError - 2){

            if(auto == Auto.two){
                collectStraight(autoTimer, drive, Collection.intakeHeightState.secondAndHalf, Collection.intakeHeightState.firstPixel, 2);
            } else if (auto == Auto.four) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.secondAndHalf, Collection.intakeHeightState.firstPixel, 1.5);
            }else if (auto == Auto.six) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.secondAndHalf, Collection.intakeHeightState.firstPixel, 0.8);
            }
            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

        }
    }

    public void delivery_and_collect_6() throws InterruptedException{

        odometry.update();
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

        if (build == Build.notBuilt){

            follower.setPath(collectSecond.followablePath, collectSecond.pathingVelocity);

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

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                collection.updateIntakeHeight();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

                delivery.ArmExtension.setPosition(delivery.ArmExtensionHome);

            }

            if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                gripperControl.reset();

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

                delivering = true;

                armOver = false;

                gotTwo = true;

            }

        }

        if (collection.getIntakeCurrentUse() > intakeNormal && !reversingIntake){
            reversingIntake = true;
            reverseIntakeTimer.reset();
            previousState = collection.getPowerState();
            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();
        }

        if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
            collection.setState(previousState);
            collection.updateIntakeState();
            reversingIntake = false;
        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() < 28000 && deliverySlides.getCurrentposition() < 30) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() > 28000 && deliverySlides.getCurrentposition() < 30) {

                sleep(400);

                drive.setAllPower(0);

                phase = Phase.finished;

            }

            if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPosition.getX()){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                armOver = true;

            }

            if (sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200){

                drive.setAllPower(0.4);

                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                sensorTouched = true;

                phase = Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
                }


            }

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 5 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){

                drive.setAllPower(0.4);

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                phase = Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
                }

            }

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();

            }

            if (Math.abs(DeliveryEndpoint.getX() - odometry.X) < 4 && Math.abs(DeliveryEndpoint.getY() - odometry.Y) < 4 && !pathing) {

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                deliverySlides.DeliverySlides(0, -1);

                pathing = true;

                gotTwo = false;

                armOver = false;

                phase = Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                }

            }

        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2, 20);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 1.5, 25);
            }


        }else if (Math.abs(lastStack.getX() - odometry.X) < collectionError && Math.abs(lastStack.getY() - odometry.Y) < collectionError - 2){

            if(auto == Auto.two){
                collectStraight(autoTimer, drive, Collection.intakeHeightState.fifthPixel, Collection.intakeHeightState.forthPixel, 2);
            } else if (auto == Auto.four) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.fifthPixel, Collection.intakeHeightState.forthPixel, 1.5);
            }else if (auto == Auto.six) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.fifthPixel, Collection.intakeHeightState.forthPixel, 0.8);
            }

            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

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

    Blue_Far_Stage.Phase phase = Phase.preload;

    Blue_Far_Stage.Build build = Build.notBuilt;

    Blue_Far_Stage.Auto auto = Blue_Far_Stage.Auto.preload;

    Preload preload = Preload.purple;

    ElapsedTime buildPaths = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        buildPaths.reset();

        while(!lockIn){

            telemetry.addData("Auto activated", auto);
            telemetry.addData("press y for 2+1", "");
            telemetry.addData("press a for 2+3", "");
            telemetry.addData("press b for 2+5", "");
            telemetry.addData("press left bumper for 2+7", "");
            telemetry.addData("press x to lock in!!!!", "");
            telemetry.update();

            if (gamepad1.a){
                auto = Auto.two;
            } else if (gamepad1.b) {
                auto = Auto.four;
            } else if (gamepad1.y) {
                auto = Auto.preload;
            }else if (gamepad1.left_bumper) {
                auto = Auto.six;
            }else if (gamepad1.x) {
                lockIn = true;
            }

        }

        while(opModeInInit()){

            if (buildPaths.seconds() > 2 && propPos == 1){

                clearAll();

                purple.threePoints(DPS1F, DPC1F, DPE1F, true, 0.65);

                yellow.twoPoints(DYS2F, DYE2F);
                yellow.threePoints(DYS3F, DYC3F, DYE3F, true, 1);

                collect.threePoints(CS1F, CC1F, CE1F);
                collect.twoPoints(CS2F, CE2F);
                collect.twoPoints(CS3F, CE3F, true, 0.55);

                collectSecond.threePoints(CS1F, CC1F, CE1F);
                collectSecond.twoPoints(CS2F, CE2F);
                collectSecond.threePoints(CS3FS, CC3FS, CE3FS, true, 0.6);

                deliver.threePoints(DS1F, DC1F, DE1F);
                deliver.twoPoints(DS2F, DE2F);
                deliver.threePoints(DS3F, DC3F, DE3F, true);

                DeliveryEndpoint = DE3F;
                CollectionEndpoint = CE3F;
                lastStack = CE3FS;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 2){

                clearAll();

                purple.threePoints(DPS1S, DPC1S, DPE1S, true, 0.8);

                yellow.twoPoints(DYS2S, DYE2S);
                yellow.threePoints(DYS3S, DYC3S, DYE3S, true, 1);

                collect.threePoints(CS1S, CC1S, CE1S);
                collect.twoPoints(CS2S, CE2S);
                collect.twoPoints(CS3S, CE3S, true, 0.55);

                collectSecond.threePoints(CS1S, CC1S, CE1S);
                collectSecond.twoPoints(CS2S, CE2S);
                collectSecond.threePoints(CS3SS, CC3SS, CE3SS, true, 0.6);

                deliver.threePoints(DS1S, DC1S, DE1S);
                deliver.twoPoints(DS2S, DE2S);
                deliver.threePoints(DS3S, DC3S, DE3S, true);

                DeliveryEndpoint = DE3S;
                CollectionEndpoint = CE3S;
                lastStack = CE3SS;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 3){

                clearAll();

                purple.twoPoints(DPS1T, DPE1T, true, 1.4);

                whitePixelSeg1.threePoints(DPS2T, DPC2T, DPE2T, true, 1);

                whitePixelSeg2.twoPoints(DYS1T, DYE1T, true, 1);

                yellow.twoPoints(DYS2T, DYE2T);
                yellow.threePoints(DYS3T, DYC3T, DYE3T, true, 0.9);

                collect.threePoints(CS1T, CC1T, CE1T);
                collect.twoPoints(CS2T, CE2T);
                collect.twoPoints(CS3T, CE3T, true, 0.55);

                collectSecond.threePoints(CS1T, CC1T, CE1T);
                collectSecond.twoPoints(CS2T, CE2T);
                collectSecond.threePoints(CS3TS, CC3TS, CE3TS, true, 0.6);

                deliver.threePoints(DS1T, DC1T, DE1T);
                deliver.twoPoints(DS2T, DE2T);
                deliver.threePoints(DS3T, DC3T, DE3T, true);

                DeliveryEndpoint = DE3T;
                CollectionEndpoint = CE3T;
                lastStack = CE3TS;

                buildPaths.reset();
            }

        }

        waitForStart();

        autoTimer.reset();

        if (propPos == 1){

            collection.setIntakeHeight(Collection.intakeHeightState.stowedMiddle);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Far_Stage.Phase.finished)){

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
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();

                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < 30 && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        targetHeading = 180;

                                        collection.setIntakeHeight(Collection.intakeHeightState.stowedMiddle);
                                        collection.updateIntakeHeight();
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

                                    if (Math.abs(extendSlidesDelivery.getX() - odometry.X) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        deliverySlides.DeliverySlides(slidesPosYellowPixel+200, 1);

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                    if (deliverySlides.getCurrentposition() > 150){

                                        delivery.setArmTargetState(Delivery.armState.delivery);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                                    }

                                    if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.off);
                                        collection.updateIntakeState();

                                    }

                                    if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.reversed);
                                        collection.updateIntakeState();

                                    }

                                    if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                }else if (Math.abs(DYE3F.getX() - odometry.X) < deliveryError && Math.abs(DYE3F.getY() - odometry.Y) < deliveryError && !pathing){


                                    drive.setAllPower(0);

                                    if (sensors.armSensor.isPressed()){

                                        sleep(200);

                                        delivery.setGripperState(Delivery.GripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);

                                    }else {

                                        while (delivery.getMainPivotPosition() < 0.95 && !(sensors.armSensor.isPressed())){
                                            delivery.setMainPivot(delivery.getMainPivotPosition() + 0.008);

                                            sleep(10);
                                        }

                                        sleep(50);

                                        delivery.setGripperState(Delivery.GripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);
                                    }

                                    if (auto == Auto.preload){

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        while (deliverySlides.getCurrentposition() > 20){
                                            delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow1Blue, odometry);
                                        }

                                        phase = Phase.finished;

                                    }else {

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                        phase = Phase.first2;

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

            collection.setIntakeHeight(Collection.intakeHeightState.stowedMiddle);
            collection.updateIntakeHeight();

            // while loop to keep auto running while tasks are not complete
            while (!(phase == Blue_Far_Stage.Phase.finished)){

                switch (phase){

                    //preload phase of auto
                    case preload:
                        switch (preload){

                            //purple preload phase of auto
                            case purple:

                                odometry.update();

                                //start pathing for auto
                                if (build == Build.notBuilt){

                                    //set follower path to purple path that was built before
                                    follower.setPath(purple.followablePath, purple.pathingVelocity);

                                    //start pathing
                                    pathing = true;

                                    //only run this if code once per path change
                                    build = Build.built;

                                    //make sure target heading aligns to start heading
                                    targetHeading = 90;

                                    //make sure yellow pixel is secure
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();

                                }

                                if (pathing){

                                    //pathing method, pass in odometry and drivetrain objects as parameters
                                    pathing = follower.followPathAuto(targetHeading, odometry, drive);

                                    //turn to leave purple pixel on spike mark
                                    if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < HeadingControlError && targetHeading == 90){
                                        targetHeading = 180;
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

                                    follower.setPath(yellow.followablePath, yellow.pathingVelocity);

                                    pathing = true;

                                    build = Build.built;

                                    targetHeading = 180;

                                }

                                if (pathing){

                                    pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);

                                    if (Math.abs(extendSlidesDelivery.getX() - odometry.X) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                    if (deliverySlides.getCurrentposition() > 150){

                                        delivery.setArmTargetState(Delivery.armState.delivery);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                    }

                                    if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.off);
                                        collection.updateIntakeState();

                                    }

                                    if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.reversed);
                                        collection.updateIntakeState();

                                    }

                                    if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                }else if (Math.abs(DYE3S.getX() - odometry.X) < deliveryError && Math.abs(DYE3S.getY() - odometry.Y) < deliveryError && !pathing){


                                    drive.setAllPower(0);

                                    if (sensors.armSensor.isPressed()){

                                        sleep(200);

                                        delivery.setGripperState(Delivery.GripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);

                                    }else {

                                        while (delivery.getMainPivotPosition() < 0.95 && !(sensors.armSensor.isPressed())){
                                            delivery.setMainPivot(delivery.getMainPivotPosition() + 0.008);

                                            sleep(10);
                                        }

                                        sleep(50);

                                        delivery.setGripperState(Delivery.GripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);
                                    }

                                    if (auto == Auto.preload){

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        while (deliverySlides.getCurrentposition() > 20){
                                            delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow2Blue, odometry);
                                        }

                                        phase = Phase.finished;

                                    }else {

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                        phase = Phase.first2;

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

            if (auto == Auto.six){
                auto = Auto.four;
            }

            collection.setIntakeHeight(Collection.intakeHeightState.stowedMiddle);
            collection.updateIntakeHeight();

            double p = 0.015;

            while (!(phase == Blue_Far_Stage.Phase.finished)){

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
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();
                                }

                                if (pathing){

                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, p, 4);

                                    if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                        targetHeading = 300;
                                    }

                                } else if (Math.abs(DPE1T.getX() - odometry.X) < collectionError && Math.abs(DPE1T.getY() - odometry.Y) < collectionError && !pathing){

                                    follower.setPath(whitePixelSeg1.followablePath, whitePixelSeg1.pathingVelocity);

                                    pathing = true;

                                    targetHeading = 270;

                                }else if (Math.abs(DPE2T.getX() - odometry.X) < collectionError && Math.abs(DPE2T.getY() - odometry.Y) < collectionError && !pathing){

                                    follower.setPath(whitePixelSeg2.followablePath, whitePixelSeg2.pathingVelocity);

                                    pathing = true;

                                    targetHeading = 180;

                                    p = 0.025;

                                }  else if (Math.abs(DYE1T.getX() - odometry.X) < collectionError && Math.abs(DYE1T.getY() - odometry.Y) < collectionError && !pathing){

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

                                    if (Math.abs(extendSlidesDelivery.getX() - odometry.X) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                    if (deliverySlides.getCurrentposition() > 150){

                                        delivery.setArmTargetState(Delivery.armState.readyForDelivering);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                                    }

                                    if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.off);
                                        collection.updateIntakeState();

                                    }

                                    if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                        collection.setState(Collection.intakePowerState.reversed);
                                        collection.updateIntakeState();

                                    }

                                    if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                }else if (Math.abs(DYE3T.getX() - odometry.X) < deliveryError && Math.abs(DYE3T.getY() - odometry.Y) < deliveryError && !pathing){

                                    drive.setAllPower(0);

                                    if (sensors.armSensor.isPressed()){

                                        sleep(200);

                                        delivery.setGripperState(Delivery.GripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);

                                    }else {

                                        while (delivery.getMainPivotPosition() < 0.95 && !(sensors.armSensor.isPressed())){
                                            delivery.setMainPivot(delivery.getMainPivotPosition() + 0.008);

                                            sleep(10);
                                        }

                                        sleep(50);

                                        delivery.setGripperState(Delivery.GripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);
                                    }

                                    if (auto == Auto.preload){

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        while (deliverySlides.getCurrentposition() > 20){
                                            delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow3Blue, odometry);
                                        }

                                        phase = Phase.finished;

                                    }else {

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                        phase = Phase.first2;

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
