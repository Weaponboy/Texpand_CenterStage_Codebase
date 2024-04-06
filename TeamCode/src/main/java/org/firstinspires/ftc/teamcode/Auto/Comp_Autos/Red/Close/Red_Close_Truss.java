package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Red.Close;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@Autonomous(name = "Red_Close_Truss", group = "red auto's")
public class Red_Close_Truss extends LinearOpMode implements CycleMethods {

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
    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1F = new Vector2D(getRealCoords(225), getRealCoords(285));
    Vector2D DPCT1F = new Vector2D(getRealCoords(165), getRealCoords(255));
    Vector2D DPE1F = new Vector2D(getRealCoords(310), getRealCoords(245));

    /**delivery and collection points*/
    Vector2D DS1F = new Vector2D(getRealCoords(44), getRealCoords(270));
    Vector2D DC1F = new Vector2D(getRealCoords(53), getRealCoords(329));
    Vector2D DE1F = new Vector2D(getRealCoords(119), getRealCoords(321));

    Vector2D DS2F = DE1F;
    Vector2D DE2F = new Vector2D(getRealCoords(181), getRealCoords(322));

    //segment 3
    Vector2D DS3F = DE2F;
    Vector2D DC3F = new Vector2D(getRealCoords(220), getRealCoords(307));
    Vector2D DE3F = new Vector2D(getRealCoords(320), getRealCoords(293));

    /**collecting paths*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(270));
    Vector2D CC1F = new Vector2D(getRealCoords(265), getRealCoords(327));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(322));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(119), getRealCoords(327));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(78), getRealCoords(320));
    Vector2D CE3F = new Vector2D(getRealCoords(33), getRealCoords(265));

    Vector2D CS3FS = CE2F;
    Vector2D CC3FS = new Vector2D(getRealCoords(78), getRealCoords(320));
    Vector2D CE3FS = new Vector2D(getRealCoords(35), getRealCoords(235));

    /**
     * second pos
     * */
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(289));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(298));
    Vector2D DPE1S = new Vector2D(getRealCoords(310), getRealCoords(264));

    /**
     * Third position
     * */
    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(337));
    Vector2D DPC1T = new Vector2D(getRealCoords(241), getRealCoords(318));
    Vector2D DPCT1T = new Vector2D(getRealCoords(188), getRealCoords(282));
    Vector2D DPE1T = new Vector2D(getRealCoords(310), getRealCoords(284));

    /**Action points*/
    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(140), getRealCoords(32));
    Vector2D armOverPosition = new Vector2D(getRealCoords(180), getRealCoords(180));

    //collection
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(183), getRealCoords(328));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(125), getRealCoords(328));
    Vector2D reverseIntake = new Vector2D(getRealCoords(72), getRealCoords(328));
    Vector2D closeGrippers = new Vector2D(getRealCoords(60), getRealCoords(310));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(245), getRealCoords(300));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(220), getRealCoords(320));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(241), getRealCoords(274));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(214), getRealCoords(300));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(227), getRealCoords(255));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(210), getRealCoords(320));

    Vector2D DeliveryEndpoint;
    Vector2D CollectionEndpoint;
    Vector2D secondStack;

    GenMethods preloadPaths = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods collectSecondStack = new GenMethods();

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

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

    int slidesPosWhitePixels = 1000;

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

    Red_Close_Truss.Phase phase = Phase.preload;

    Red_Close_Truss.Build build = Build.notBuilt;

    Red_Close_Truss.Auto auto = Red_Close_Truss.Auto.preload;

    Collection.intakePowerState previousState = Collection.intakePowerState.off;

    ElapsedTime buildPaths = new ElapsedTime();

    ElapsedTime reverseIntakeTimer = new ElapsedTime();

    ElapsedTime extendSlidesPreload = new ElapsedTime();

    ElapsedTime autoTimer = new ElapsedTime();

    public void delivery_and_collect_2() throws InterruptedException {

        odometry.update();
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

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
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
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

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();

            }

            if (Math.abs(closeGrippers.getX() - odometry.X) < IntakeControlError && Math.abs(closeGrippers.getY() - odometry.Y) < (IntakeControlError+20)){

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
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.two) {

                    phase = Phase.finished;

                    deliverySlides.DeliverySlides(0, -0.2);

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
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
            delivery.setGripperState(Delivery.GripperState.open);
            delivery.updateGrippers();

            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            collection.updateIntakeHeight();

            drive.setAllPower(0);

            collection.setState(Collection.intakePowerState.on);
            collection.updateIntakeState();

            boolean collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

            int counter = 0;

            while (autoTimer.milliseconds() < 26000 && !collectionDone){

                counter++;
                collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

                if (counter <= 6){

                    sleep(50);

                } else if (counter > 6 && counter <= 12) {

                    collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
                    collection.updateIntakeHeight();

                    sleep(50);

                } else if (counter == 13){

                    drive.strafeLeft();
                    sleep(200);
                    drive.setAllPower(0);

                } else if (counter == 14){

                    drive.strafeRight();
                    sleep(200);
                    drive.setAllPower(0);

                } else if (counter > 14 && counter <= 20) {

                    sleep(50);

                } else if (counter > 20) {

                    collectionDone = true;

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

//        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

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

                collection.setIntakeHeight(Collection.intakeHeightState.thirdAndHalf);
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

                if (auto == Auto.four) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
                    }

                } else {

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
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.four) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
                    }

                } else {

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

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();

            }

            if (Math.abs(closeGrippers.getX() - odometry.X) < IntakeControlError && Math.abs(closeGrippers.getY() - odometry.Y) < (IntakeControlError+20)){

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
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                deliverySlides.DeliverySlides(0, -1);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Auto.four) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
                    }

                } else {

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
            delivery.setGripperState(Delivery.GripperState.open);
            delivery.updateGrippers();

            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            collection.updateIntakeHeight();

            drive.setAllPower(0);

            collection.setState(Collection.intakePowerState.on);
            collection.updateIntakeState();

            boolean collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

            int counter = 0;

            while (autoTimer.milliseconds() < 26000 && !collectionDone){

                counter++;
                collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

                if (counter <= 6){

                    sleep(50);

                } else if (counter > 6 && counter <= 12) {

                    collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
                    collection.updateIntakeHeight();

                    sleep(50);

                } else if (counter == 13){

                    drive.strafeLeft();
                    sleep(200);
                    drive.setAllPower(0);

                } else if (counter == 14){

                    drive.strafeRight();
                    sleep(200);
                    drive.setAllPower(0);

                } else if (counter > 14 && counter <= 20) {

                    sleep(50);

                } else if (counter > 20) {

                    collectionDone = true;

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

            follower.setPath(collectSecondStack.followablePath, collectSecondStack.pathingVelocity);

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

            if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() < 28000 && deliverySlides.getCurrentposition() < 50) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() > 28000 && deliverySlides.getCurrentposition() < 50) {

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

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 10 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){

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

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < (IntakeControlError+20)){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.reversed);
                collection.updateIntakeState();

            }

            if (Math.abs(closeGrippers.getX() - odometry.X) < IntakeControlError && Math.abs(closeGrippers.getY() - odometry.Y) < (IntakeControlError+20)){

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
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

                deliverySlides.DeliverySlides(0, -1);

                pathing = true;

                gotTwo = false;

                armOver = false;

                phase = Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
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
            delivery.setGripperState(Delivery.GripperState.open);
            delivery.updateGrippers();

            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            collection.updateIntakeHeight();

            drive.setAllPower(0);

            collection.setState(Collection.intakePowerState.on);
            collection.updateIntakeState();

            boolean collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

            int counter = 0;

            while (autoTimer.milliseconds() < 26000 && !collectionDone){

                counter++;
                collectionDone = !sensors.LeftClawSensor.isPressed() && !sensors.RightClawSensor.isPressed();

                if (counter <= 6){

                    sleep(50);

                } else if (counter > 6 && counter <= 12) {

                    collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
                    collection.updateIntakeHeight();

                    sleep(50);

                } else if (counter == 13){

                    drive.strafeLeft();
                    sleep(200);
                    drive.setAllPower(0);

                } else if (counter == 14){

                    drive.strafeRight();
                    sleep(200);
                    drive.setAllPower(0);

                } else if (counter > 14 && counter <= 20) {

                    sleep(50);

                } else if (counter > 20) {

                    collectionDone = true;

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

            }

            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

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
                collect.threePoints(CS3F, CC3F, CE3F, true, 0.45);

                collectSecondStack.threePoints(CS1F, CC1F, CE1F);
                collectSecondStack.twoPoints(CS2F, CE2F);
                collectSecondStack.threePoints(CS3FS, CC3FS, CE3FS, true, 0.4);

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

                collect.threePoints(CS1F, CC1F, CE1F);
                collect.twoPoints(CS2F, CE2F);
                collect.threePoints(CS3F, CC3F, CE3F, true, 0.45);

                collectSecondStack.threePoints(CS1F, CC1F, CE1F);
                collectSecondStack.twoPoints(CS2F, CE2F);
                collectSecondStack.threePoints(CS3FS, CC3FS, CE3FS, true, 0.4);

                deliver.threePoints(DS1F, DC1F, DE1F);
                deliver.twoPoints(DS2F, DE2F);
                deliver.threePoints(DS3F, DC3F, DE3F, true);

                DeliveryEndpoint = DE3F;
                CollectionEndpoint = CE3F;
                secondStack = CE3FS;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 3){

                clearAll();

                preloadPaths.fourPoints(DPS1T, DPC1T, DPCT1T, DPE1T, true, 0.45);

                collect.threePoints(CS1F, CC1F, CE1F);
                collect.twoPoints(CS2F, CE2F);
                collect.threePoints(CS3F, CC3F, CE3F, true, 0.45);

                collectSecondStack.threePoints(CS1F, CC1F, CE1F);
                collectSecondStack.twoPoints(CS2F, CE2F);
                collectSecondStack.threePoints(CS3FS, CC3FS, CE3FS, true, 0.4);

                deliver.threePoints(DS1F, DC1F, DE1F);
                deliver.twoPoints(DS2F, DE2F);
                deliver.threePoints(DS3F, DC3F, DE3F, true);

                DeliveryEndpoint = DE3F;
                CollectionEndpoint = CE3F;
                secondStack = CE3FS;

                buildPaths.reset();
            }

        }

        waitForStart();

        if (propPos == 1){

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            extendSlidesPreload.reset();

            double p = 0.02;

            while (!(phase == Red_Close_Truss.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 90;
                        }

                        if (extendSlidesPreload.milliseconds() > 400 && deliverySlides.getCurrentposition() < 50){
                            deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, p, 2);

                            if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < 30 && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && targetHeading == 90){

                                targetHeading = 5;

                                collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                collection.updateIntakeHeight();

                            }

                            if (Math.abs(oneEightyHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingF.getY() - odometry.Y) < 40){

                                p = 0.01;

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

                        if (build == Red_Close_Truss.Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Red_Close_Truss.Build.built;
                            targetHeading = 90;

                        }

                        if (extendSlidesPreload.milliseconds() > 400 && deliverySlides.getCurrentposition() < 50){
                            deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.008, 2);

                            if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < 30 && targetHeading == 90){

                                targetHeading = 40;

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

                            if (auto == Red_Close_Truss.Auto.preload){

                                delivery.setArmTargetState(Delivery.armState.collect);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                deliverySlides.DeliverySlides(0, -0.5);

                                while (deliverySlides.getCurrentposition() > 20){
                                    delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow2Blue, odometry);
                                }

                                phase = Red_Close_Truss.Phase.finished;

                            }else {

                                delivery.setArmTargetState(Delivery.armState.collect);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                deliverySlides.DeliverySlides(0, -0.5);

                                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                phase = Red_Close_Truss.Phase.first2;

                                build = Red_Close_Truss.Build.notBuilt;

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

            while (!(phase == Red_Close_Truss.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow3Blue, odometry);

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 90;
                        }

                        if (extendSlidesPreload.milliseconds() > 400 && deliverySlides.getCurrentposition() < 50){
                            deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.009);

                            if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 90){
                                targetHeading = 50;
                            }

                            if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingT.getY() - odometry.Y) < 30){

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

        sensors.initAprilTag(telemetry, true);

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

    }

}
