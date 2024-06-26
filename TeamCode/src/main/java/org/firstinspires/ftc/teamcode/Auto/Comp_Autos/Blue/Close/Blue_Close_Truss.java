package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue.Close;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Red.Close.Red_Close_Stage;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Red.Close.Red_Close_Truss;
import org.firstinspires.ftc.teamcode.Auto.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@Autonomous(name = "Blue_Close_Truss", group = "blue auto's")
public class Blue_Close_Truss extends LinearOpMode implements CycleMethods {

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
    Vector2D DPE1F = new Vector2D(getRealCoords(311), getRealCoords(76));

    /**delivery and collection points*/
    Vector2D DS1F = new Vector2D(getRealCoords(44), getRealCoords(92));
    Vector2D DC1F = new Vector2D(getRealCoords(53), getRealCoords(31));
    Vector2D DE1F = new Vector2D(getRealCoords(119), getRealCoords(35));

    Vector2D DS2F = DE1F;
    Vector2D DE2F = new Vector2D(getRealCoords(210), getRealCoords(34));

    //segment 3
    Vector2D DS3F = DE2F;
    Vector2D DC3F = new Vector2D(getRealCoords(220), getRealCoords(53));
    Vector2D DE3F = new Vector2D(getRealCoords(320), getRealCoords(62));

    /**collecting paths*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(265), getRealCoords(33));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(34));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(119), getRealCoords(33));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(81), getRealCoords(40));
    Vector2D CE3F = new Vector2D(getRealCoords(40), getRealCoords(90));

    Vector2D CS3FS = CE2F;
    Vector2D CC3FS = new Vector2D(getRealCoords(84), getRealCoords(40));
    Vector2D CE3FS = new Vector2D(getRealCoords(40), getRealCoords(120));

    /**
     * second pos
     * */
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(75));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(66));
    Vector2D DPE1S = new Vector2D(getRealCoords(312), getRealCoords(89));

    /**delivery and collection points*/

    Vector2D DS1S = new Vector2D(getRealCoords(44), getRealCoords(92));
    Vector2D DC1S = new Vector2D(getRealCoords(53), getRealCoords(31));
    Vector2D DE1S = new Vector2D(getRealCoords(119), getRealCoords(35));

    Vector2D DS2S = DE1S;
    Vector2D DE2S = new Vector2D(getRealCoords(181), getRealCoords(34));

    //segment 3
    Vector2D DS3S = DE2S;
    Vector2D DC3S = new Vector2D(getRealCoords(220), getRealCoords(53));
    Vector2D DE3S = new Vector2D(getRealCoords(320), getRealCoords(58));

    /**collecting paths*/
    Vector2D CS1S = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1S = new Vector2D(getRealCoords(265), getRealCoords(33));
    Vector2D CE1S = new Vector2D(getRealCoords(180), getRealCoords(33));

    Vector2D CS2S = CE1S;
    Vector2D CE2S = new Vector2D(getRealCoords(119), getRealCoords(33));

    Vector2D CS3S = CE2S;
    Vector2D CC3S = new Vector2D(getRealCoords(81), getRealCoords(40));
    Vector2D CE3S = new Vector2D(getRealCoords(40), getRealCoords(90));

    Vector2D CS3SS = CE2S;
    Vector2D CC3SS = new Vector2D(getRealCoords(84), getRealCoords(40));
    Vector2D CE3SS = new Vector2D(getRealCoords(42), getRealCoords(120));

    /**
     * Third position
     * */
    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(225), getRealCoords(75));
    Vector2D DPCT1T = new Vector2D(getRealCoords(165), getRealCoords(105));
    Vector2D DPE1T = new Vector2D(getRealCoords(314), getRealCoords(106));

    /**delivery and collection points*/
    Vector2D DS1T = new Vector2D(getRealCoords(44), getRealCoords(92));
    Vector2D DC1T = new Vector2D(getRealCoords(53), getRealCoords(31));
    Vector2D DE1T = new Vector2D(getRealCoords(119), getRealCoords(36));

    Vector2D DS2T = DE1T;
    Vector2D DE2T = new Vector2D(getRealCoords(181), getRealCoords(35));

    //segment 3
    Vector2D DS3T = DE2T;
    Vector2D DC3T = new Vector2D(getRealCoords(220), getRealCoords(53));
    Vector2D DE3T = new Vector2D(getRealCoords(320), getRealCoords(64));

    /**collecting paths*/
    Vector2D CS1T = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1T = new Vector2D(getRealCoords(265), getRealCoords(33));
    Vector2D CE1T = new Vector2D(getRealCoords(180), getRealCoords(37));

    Vector2D CS2T = CE1T;
    Vector2D CE2T = new Vector2D(getRealCoords(119), getRealCoords(35));

    Vector2D CS3T = CE2T;
    Vector2D CC3T = new Vector2D(getRealCoords(81), getRealCoords(40));
    Vector2D CE3T = new Vector2D(getRealCoords(43), getRealCoords(95));

    Vector2D CS3TS = CE2T;
    Vector2D CC3TS = new Vector2D(getRealCoords(84), getRealCoords(40));
    Vector2D CE3TS = new Vector2D(getRealCoords(44), getRealCoords(125));

    /**Action points*/
    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(120), getRealCoords(32));
    Vector2D extendSlidesDeliverySafe = new Vector2D(getRealCoords(150), getRealCoords(32));
    Vector2D armOverPosition = new Vector2D(getRealCoords(160), getRealCoords(180));
    Vector2D armOverPositionSafe = new Vector2D(getRealCoords(190), getRealCoords(180));

    //collection
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(183), getRealCoords(32));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(105), getRealCoords(32));
    Vector2D reverseIntake = new Vector2D(getRealCoords(45), getRealCoords(80));
    Vector2D restartIntake = new Vector2D(getRealCoords(65), getRealCoords(32));

    //first position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(245), getRealCoords(60));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(220), getRealCoords(80));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(241), getRealCoords(86));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(214), getRealCoords(70));

    //third position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(227), getRealCoords(105));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(210), getRealCoords(60));

    Vector2D DeliveryEndpoint;
    Vector2D CollectionEndpoint;
    Vector2D secondStack;

    GenMethods preloadPaths = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods collectSecondStack = new GenMethods();

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

    boolean armOver = true;

    boolean closeRight = false;

    boolean closeLeft = false;

    boolean reversingIntake = false;

    boolean sensorTouched = false;

    /**doubles*/
    double targetHeading = 0;

    double timeChanger;

    double intakeCurrentOne = 5500;

    double intakeNormal = 5500;

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

    int slidesPosWhitePixels = 1200;

    int slidesPosYellowPixel = 400;

    public void clearAll(){
        preloadPaths.clearAll();
        collect.clearAll();
        deliver.clearAll();
        collectSecondStack.clearAll();
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

    enum Build{
        built,
        notBuilt
    }

    Blue_Close_Truss.Phase phase = Phase.preload;

    Blue_Close_Truss.Build build = Build.notBuilt;

    Blue_Close_Truss.Auto auto = Blue_Close_Truss.Auto.preload;

    Collection.intakePowerState previousState = Collection.intakePowerState.off;

    ElapsedTime buildPaths = new ElapsedTime();

    ElapsedTime headingPos1 = new ElapsedTime();

    ElapsedTime extendSlidesPreload = new ElapsedTime();

    ElapsedTime autoTimer = new ElapsedTime();

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

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() < -100) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDeliverySafe.getX()) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPosition.getX() && odometry.getVerticalVelocity() < -100){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                armOver = true;

            } else if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPositionSafe.getX()){

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

                    deliverySlides.DeliverySlides(0, -0.5);

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

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 5 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){

                drive.setAllPower(0.4);

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

                    drive.setAllPower(0);

                    deliverySlides.DeliverySlides(0, -0.5);

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

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < 20+IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError)){

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

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.two) {

                    phase = Phase.finished;

                    deliverySlides.DeliverySlides(0, -0.5);

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
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 1.5, 50);
            }

        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError - 2){

            if(auto == Auto.two){
                collectStraight(autoTimer, drive, Collection.intakeHeightState.forthAndHalf, Collection.intakeHeightState.forthPixel, 4);
            } else if (auto == Auto.four) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.forthAndHalf, Collection.intakeHeightState.forthPixel, 2);
            }else{
                collectStraight(autoTimer, drive, Collection.intakeHeightState.forthAndHalf, Collection.intakeHeightState.forthPixel);
            }
            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            collection.updateIntakeHeight();

        }

    }

    public void delivery_and_collect_4() throws InterruptedException{

        odometry.update();

//        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());
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
//
//            if (collection.getIntakeCurrentUse() > 5500 && !reversingIntake){
//                reversingIntake = true;
//                reverseIntakeTimer.reset();
//                previousState = collection.getPowerState();
//                collection.setState(Collection.intakePowerState.reversed);
//                collection.updateIntakeState();
//            }
//
//            if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
//                collection.setState(previousState);
//                collection.updateIntakeState();
//                reversingIntake = false;
//            }

        }
//        if (gripperControl.milliseconds() > 400 && gripperControl.milliseconds() < 500 && gotTwo){
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//        }
//
//        if (gripperControl.milliseconds() > 500 && gripperControl.milliseconds() < 700 && gotTwo){
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//            collection.setState(Collection.intakePowerState.reversed);
//            collection.updateIntakeState();
//
//        }
//
//        if (gripperControl.milliseconds() > 700 && gripperControl.milliseconds() < 900 && gotTwo){
//
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() < -100) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDeliverySafe.getX()) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPosition.getX() && odometry.getVerticalVelocity() < -100){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                armOver = true;

            } else if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPositionSafe.getX()){

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

                if (auto == Auto.four) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    deliverySlides.DeliverySlides(0, -0.5);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    deliverySlides.DeliverySlides(0, -0.5);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Phase.third2;

                }

            }

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 5 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){

                drive.setAllPower(0.4);
                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.four) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    deliverySlides.DeliverySlides(0, -0.5);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    deliverySlides.DeliverySlides(0, -0.5);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Phase.third2;

                }

            }

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < 20+IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError)){

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

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.four) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    deliverySlides.DeliverySlides(0, -0.5);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    deliverySlides.DeliverySlides(0, -0.5);

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Phase.third2;

                }
            }

        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2, 20);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 1.5, 50);
            }


        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError - 2){

            if(auto == Auto.two){
                collectStraight(autoTimer, drive, Collection.intakeHeightState.secondAndHalf, Collection.intakeHeightState.firstPixel, 4);
            } else if (auto == Auto.four) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.secondAndHalf, Collection.intakeHeightState.firstPixel, 2);
            }else{
                collectStraight(autoTimer, drive, Collection.intakeHeightState.secondAndHalf, Collection.intakeHeightState.firstPixel);
            }

            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            collection.updateIntakeHeight();
        }
    }

    public void delivery_and_collect_6() throws InterruptedException{

        odometry.update();
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

        if (build == Build.notBuilt){

            follower.setPath(collectSecondStack.followablePath, collectSecondStack.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Build.built;

            targetHeading = 180;

            delivering = false;

            delivery.setGripperState(Delivery.GripperState.open);

            delivery.updateGrippers();

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

                drive.setAllPower(0);

                follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

                delivering = true;

                armOver = false;

                gotTwo = true;

            }

//            if (collection.getIntakeCurrentUse() > 5500 && !reversingIntake){
//                reversingIntake = true;
//                reverseIntakeTimer.reset();
//                previousState = collection.getPowerState();
//                collection.setState(Collection.intakePowerState.reversed);
//                collection.updateIntakeState();
//            }
//
//            if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
//                collection.setState(previousState);
//                collection.updateIntakeState();
//                reversingIntake = false;
//            }
//
        }
//
//        if (gripperControl.milliseconds() > 400 && gripperControl.milliseconds() < 500 && gotTwo){
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//        }
//
//        if (gripperControl.milliseconds() > 500 && gripperControl.milliseconds() < 700 && gotTwo){
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//            collection.setState(Collection.intakePowerState.reversed);
//            collection.updateIntakeState();
//
//        }
//
//        if (gripperControl.milliseconds() > 700 && gripperControl.milliseconds() < 900 && gotTwo){
//
//            collection.setState(Collection.intakePowerState.off);
//            collection.updateIntakeState();
//
//            delivery.setGripperState(Delivery.GripperState.closed);
//            delivery.updateGrippers();
//
//        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() < 28000 && deliverySlides.getCurrentposition() < 30 && odometry.getVerticalVelocity() < -100) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDeliverySafe.getX() && autoTimer.milliseconds() < 28000 && deliverySlides.getCurrentposition() < 30) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() > 28000 && deliverySlides.getCurrentposition() < 30) {

                sleep(400);

                drive.setAllPower(0);

                phase = Phase.finished;
            }

            if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPosition.getX() && odometry.getVerticalVelocity() < -100){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                armOver = true;

            } else if (deliverySlides.getCurrentposition() > 150 && !armOver && odometry.X > armOverPositionSafe.getX()){

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

                phase = Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
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

                phase = Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                }

            }

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < (IntakeControlError+20)) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < 20+IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError)){

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
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 1.5, 50);
            }


        }else if (Math.abs(secondStack.getX() - odometry.X) < collectionError && Math.abs(secondStack.getY() - odometry.Y) < collectionError - 2){

            if(auto == Auto.two){
                collectStraight(autoTimer, drive, Collection.intakeHeightState.forthAndHalf, Collection.intakeHeightState.forthPixel, 2);
            } else if (auto == Auto.four) {
                collectStraight(autoTimer, drive, Collection.intakeHeightState.forthAndHalf, Collection.intakeHeightState.forthPixel, 1.5);
            }else{
                collectStraight(autoTimer, drive, Collection.intakeHeightState.forthAndHalf, Collection.intakeHeightState.forthPixel);
            }
            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            collection.updateIntakeHeight();

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
            telemetry.addData("press left bumper for 2+6", "");
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

                preloadPaths.fourPoints(DPS1F, DPC1F, DPCT1F, DPE1F, true);

                collect.threePoints(CS1F, CC1F, CE1F);
                collect.twoPoints(CS2F, CE2F);
                collect.threePoints(CS3F, CC3F, CE3F, true, 0.9);

                collectSecondStack.threePoints(CS1F, CC1F, CE1F);
                collectSecondStack.twoPoints(CS2F, CE2F);
                collectSecondStack.threePoints(CS3FS, CC3FS, CE3FS, true, 1.2);

                deliver.threePoints(DS1F, DC1F, DE1F);
                deliver.twoPoints(DS2F, DE2F);
                deliver.threePoints(DS3F, DC3F, DE3F, true);

                DeliveryEndpoint = DE3F;
                CollectionEndpoint = CE3F;
                secondStack = CE3FS;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 2){

                clearAll();

                preloadPaths.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true);

                collect.threePoints(CS1S, CC1S, CE1S);
                collect.twoPoints(CS2S, CE2S);
                collect.threePoints(CS3S, CC3S, CE3S, true, 0.9);

                collectSecondStack.threePoints(CS1S, CC1S, CE1S);
                collectSecondStack.twoPoints(CS2S, CE2S);
                collectSecondStack.threePoints(CS3SS, CC3SS, CE3SS, true, 1.2);

                deliver.threePoints(DS1S, DC1S, DE1S);
                deliver.twoPoints(DS2S, DE2S);
                deliver.threePoints(DS3S, DC3S, DE3S, true);

                DeliveryEndpoint = DE3S;
                CollectionEndpoint = CE3S;
                secondStack = CE3SS;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 3){

                clearAll();

                preloadPaths.fourPoints(DPS1T, DPC1T, DPCT1T, DPE1T, true, 0.45);

                collect.threePoints(CS1T, CC1T, CE1T);
                collect.twoPoints(CS2T, CE2T);
                collect.threePoints(CS3T, CC3T, CE3T, true, 0.9);

                collectSecondStack.threePoints(CS1T, CC1T, CE1T);
                collectSecondStack.twoPoints(CS2T, CE2T);
                collectSecondStack.threePoints(CS3TS, CC3TS, CE3TS, true, 1.2);

                deliver.threePoints(DS1T, DC1T, DE1T);
                deliver.twoPoints(DS2T, DE2T);
                deliver.threePoints(DS3T, DC3T, DE3T, true);

                DeliveryEndpoint = DE3T;
                CollectionEndpoint = CE3T;
                secondStack = CE3TS;

                buildPaths.reset();
            }

        }

        waitForStart();

        autoTimer.reset();

        if (propPos == 1){

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            extendSlidesPreload.reset();

            while (!(phase == Blue_Close_Truss.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow1Blue, odometry);

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (extendSlidesPreload.milliseconds() > 400 && deliverySlides.getCurrentposition() < 50){
                            deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.009);

                            if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < 30 && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                targetHeading = 350;
                            }

                            if (Math.abs(oneEightyHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingF.getY() - odometry.Y) < 30){

                                targetHeading = 180;

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (deliverySlides.getCurrentposition() > 150){

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                            }

                        }else if (Math.abs(DPE1F.getX() - odometry.X) < deliveryError && Math.abs(DPE1F.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.setAllPower(0);

                            if (sensors.armSensor.isPressed()){

                                sleep(200);

                                delivery.setLeftGripperState(Delivery.leftGripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

                            }else {

                                while (delivery.getMainPivotPosition() < 0.95 && !(sensors.armSensor.isPressed())){
                                    delivery.setMainPivot(delivery.getMainPivotPosition() + 0.015);

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

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            extendSlidesPreload.reset();

            while (!(phase == Phase.finished) && opModeIsActive()){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                        if (build == Blue_Close_Truss.Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Blue_Close_Truss.Build.built;
                            targetHeading = 270;

                        }

                        if (extendSlidesPreload.milliseconds() > 400 && deliverySlides.getCurrentposition() < 50){
                            deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.008, 2);

                            if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < 30 && targetHeading == 270){

                                targetHeading = 310;

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (Math.abs(oneEightyHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingS.getY() - odometry.Y) < HeadingControlError+20){

                                targetHeading = 180;

                            }

                            if (deliverySlides.getCurrentposition() > 150){

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                            }


                        }else if (Math.abs(DPE1S.getX() - odometry.X) < deliveryError && Math.abs(DPE1S.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.setAllPower(0);

                            if (sensors.armSensor.isPressed()){

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

                            }else {

                                while (delivery.getMainPivotPosition() < 0.95 && !(sensors.armSensor.isPressed())){
                                    delivery.setMainPivot(delivery.getMainPivotPosition() + 0.01);

                                    sleep(10);
                                }

                                sleep(50);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);
                            }

                            if (auto == Blue_Close_Truss.Auto.preload){

                                delivery.setArmTargetState(Delivery.armState.collect);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                deliverySlides.DeliverySlides(0, -0.5);

                                while (deliverySlides.getCurrentposition() > 20){
                                    delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow2Blue, odometry);
                                }

                                phase = Blue_Close_Truss.Phase.finished;

                            }else {

                                delivery.setArmTargetState(Delivery.armState.collect);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                deliverySlides.DeliverySlides(0, -0.5);

                                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                phase = Blue_Close_Truss.Phase.first2;

                                build = Blue_Close_Truss.Build.notBuilt;

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

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            extendSlidesPreload.reset();

            double p = 0.02;

            while (!(phase == Blue_Close_Truss.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (extendSlidesPreload.milliseconds() > 400 && deliverySlides.getCurrentposition() < 50){
                            deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, p, 2);

                            if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < 50 && targetHeading == 270){

                                targetHeading = 330;

                                collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                collection.updateIntakeHeight();

                            }

                            if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && targetHeading == 330){

                                targetHeading = 180;

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (deliverySlides.getCurrentposition() > 150){

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                            }

                        }else if (Math.abs(DPE1T.getX() - odometry.X) < deliveryError && Math.abs(DPE1T.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.setAllPower(0);

                            if (sensors.armSensor.isPressed()){

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

                            }else {

                                while (delivery.getMainPivotPosition() < 0.95 && !(sensors.armSensor.isPressed())){
                                    delivery.setMainPivot(delivery.getMainPivotPosition() + 0.01);

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
