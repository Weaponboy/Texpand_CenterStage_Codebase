package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue.Close;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import static java.lang.Thread.sleep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Red.Close.Red_Close_Stage;
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

@Autonomous(name = "Blue_Close_Stage", group = "blue auto's")
public class Blue_Close_Stage extends LinearOpMode implements CycleMethods {

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
    Vector2D DPE1F = new Vector2D(getRealCoords(314), getRealCoords(76));

    /**delivery and collection points*/
    Vector2D DS1F = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1F = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1F = new Vector2D(getRealCoords(106), getRealCoords(152));

    Vector2D DS2F = DE1F;
    Vector2D DE2F = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3F = DE2F;
    Vector2D DC3F = new Vector2D(getRealCoords(220), getRealCoords(137));
    Vector2D DE3F = new Vector2D(getRealCoords(320), getRealCoords(120));

    /**collecting paths*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(312), getRealCoords(156));
    Vector2D CE1F = new Vector2D(getRealCoords(220), getRealCoords(154));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(40), getRealCoords(175));
    Vector2D CE3F = new Vector2D(getRealCoords(40), getRealCoords(150));

    Vector2D CS3FS = CE2F;
    Vector2D CC3FS = new Vector2D(getRealCoords(52), getRealCoords(160));
    Vector2D CE3FS = new Vector2D(getRealCoords(42), getRealCoords(120));

    /**
     * second pos
     * */
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(71));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(62));
    Vector2D DPE1S = new Vector2D(getRealCoords(310), getRealCoords(91));

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
    Vector2D DE3S = new Vector2D(getRealCoords(320), getRealCoords(118));

    /**collecting paths*/

    Vector2D CS1S = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1S = new Vector2D(getRealCoords(312), getRealCoords(162));
    Vector2D CE1S = new Vector2D(getRealCoords(220), getRealCoords(156));

    Vector2D CS2S = CE1S;
    Vector2D CE2S = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3S = CE2S;
    Vector2D CE3S = new Vector2D(getRealCoords(40), getRealCoords(150));

    Vector2D CS3SS = CE2S;
    Vector2D CC3SS = new Vector2D(getRealCoords(72), getRealCoords(153));
    Vector2D CE3SS = new Vector2D(getRealCoords(40), getRealCoords(120));

    /**
     * Third position
     * */

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(220), getRealCoords(79));
    Vector2D DPCT1T = new Vector2D(getRealCoords(160), getRealCoords(110));
    Vector2D DPE1T = new Vector2D(getRealCoords(314), getRealCoords(110));

    /**delivery and collection points*/

    //segment 1 deliver straight
    Vector2D DS1T = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1T = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1T = new Vector2D(getRealCoords(106), getRealCoords(156));

    //segment 2
    Vector2D DS2T = DE1T;
    Vector2D DE2T = new Vector2D(getRealCoords(206), getRealCoords(156));

    //segment 3
    Vector2D DS3T = DE2T;
    Vector2D DC3T = new Vector2D(getRealCoords(220), getRealCoords(137));
    Vector2D DE3T = new Vector2D(getRealCoords(320), getRealCoords(130));

    /**collecting paths*/

    Vector2D CS1T = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1T = new Vector2D(getRealCoords(312), getRealCoords(162));
    Vector2D CE1T = new Vector2D(getRealCoords(220), getRealCoords(156));

    Vector2D CS2T = CE1T;
    Vector2D CE2T = new Vector2D(getRealCoords(91), getRealCoords(156));

    Vector2D CS3T = CE2T;
    Vector2D CE3T = new Vector2D(getRealCoords(42), getRealCoords(156));

    Vector2D CS3TS = CE2T;
    Vector2D CC3TS = new Vector2D(getRealCoords(85), getRealCoords(153));
    Vector2D CE3TS = new Vector2D(getRealCoords(43), getRealCoords(125));

    /**Action points*/
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(180), getRealCoords(155));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(125), getRealCoords(154));
    Vector2D restartIntake = new Vector2D(getRealCoords(65), getRealCoords(154));
    Vector2D reverseIntake = new Vector2D(getRealCoords(45), getRealCoords(154));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(245), getRealCoords(80));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(223), getRealCoords(60));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(241), getRealCoords(86));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(214), getRealCoords(75));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(227), getRealCoords(105));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(210), getRealCoords(42));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(150), getRealCoords(180));
    Vector2D armOverPosition = new Vector2D(getRealCoords(180), getRealCoords(180));

    Vector2D DeliveryEndpoint;
    Vector2D CollectionEndpoint;

    GenMethods preloadPaths = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods collectSecondStack = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods deliverRecollect = new GenMethods();

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
        deliverRecollect.clearAll();
        collectSecondStack.clearAll();
    }

    /**enums*/

    enum Phase{
        preload,
        first2,
        second2,
        fifth1,
        third2,
        finished
    }

    enum Auto{
        preload,
        two,
        four,
        five,
        six
    }

    enum Build{
        built,
        notBuilt
    }

    Blue_Close_Stage.Phase phase = Phase.preload;

    Blue_Close_Stage.Build build = Build.notBuilt;

    Blue_Close_Stage.Auto auto = Blue_Close_Stage.Auto.preload;

    Collection.intakePowerState previousState = Collection.intakePowerState.off;

    ElapsedTime buildPaths = new ElapsedTime();

    ElapsedTime headingPos1 = new ElapsedTime();

    ElapsedTime extendSlidesPreload = new ElapsedTime();

    ElapsedTime autoTimer = new ElapsedTime();

    Vector2D secondStack = new Vector2D(getRealCoords(41), getRealCoords(115));

    public void delivery_and_collect_2() throws InterruptedException {

        odometry.update();
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

        if (build == Blue_Close_Stage.Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Blue_Close_Stage.Build.built;

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

                    deliverySlides.DeliverySlides(0, -0.5);

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

                if (auto == Auto.two) {

                    phase = Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
                    }

                } else {

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

            sleep(4000);

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

        }

    }

    public void delivery_and_collect_4() throws InterruptedException{

        odometry.update();

//        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());
        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);

        if (build == Blue_Close_Stage.Build.notBuilt){

            follower.setPath(collect.followablePath, collect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Blue_Close_Stage.Build.built;

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

                    if (auto == Auto.six){
                        phase = Phase.third2;
                    }else{
                        phase = Phase.fifth1;
                    }


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

                    if (auto == Auto.six){
                        phase = Phase.third2;
                    }else{
                        phase = Phase.fifth1;
                    }


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

                    if (auto == Auto.six){
                        phase = Phase.third2;
                    }else{
                        phase = Phase.fifth1;
                    }


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

    public void delivery_and_collect_5() throws InterruptedException{

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

                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
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

            if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() < 28000 && deliverySlides.getVelocity() < 20) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() > 28000 && deliverySlides.getVelocity() > 20) {

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
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);
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

            collectStraightOne(autoTimer, drive, Collection.intakeHeightState.secondPixel, Collection.intakeHeightState.firstPixel);

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

        if (build == Blue_Close_Stage.Build.notBuilt){

            follower.setPath(collectSecondStack.followablePath, collectSecondStack.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            pathing = true;

            build = Blue_Close_Stage.Build.built;

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

                drive.setAllPower(0);

                follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

                delivering = true;

                armOver = false;

                gotTwo = true;

            }

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() < 28000 && deliverySlides.getCurrentposition() < 10) {

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            } else if (odometry.X > extendSlidesDelivery.getX() && autoTimer.milliseconds() > 28000 && deliverySlides.getCurrentposition() < 10) {

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

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        while(!lockIn){

            telemetry.addData("Auto activated", auto);
            telemetry.addData("press y for 2+0", "");
            telemetry.addData("press a for 2+2", "");
            telemetry.addData("press b for 2+4", "");
            telemetry.addData("press left bumper for 2+6", "");
            telemetry.addData("press right bumper for 2+5", "");
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
            }else if (gamepad1.right_bumper) {
                auto = Auto.five;
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
                collect.twoPoints(CS3F, CE3F, true, 0.55);

                collectSecondStack.threePoints(CS1F, CC1F, CE1F);
                collectSecondStack.twoPoints(CS2F, CE2F);
                collectSecondStack.threePoints(CS3FS, CC3FS, CE3FS, true, 0.5);

                deliver.threePoints(DS1F, DC1F, DE1F);
                deliver.twoPoints(DS2F, DE2F);
                deliver.threePoints(DS3F, DC3F, DE3F, true);

                DeliveryEndpoint = DE3F;
                CollectionEndpoint = CE3F;
                secondStack = CE3FS;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 2){

                clearAll();

                preloadPaths.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true, 0.35);

                collect.threePoints(CS1S, CC1S, CE1S);
                collect.twoPoints(CS2S, CE2S);
                collect.twoPoints(CS3S, CE3S, true, 0.55);

                collectSecondStack.threePoints(CS1S, CC1S, CE1S);
                collectSecondStack.twoPoints(CS2S, CE2S);
                collectSecondStack.threePoints(CS3SS, CC3SS, CE3SS, true, 0.5);

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
                collect.twoPoints(CS3T, CE3T, true, 0.55);

                collectSecondStack.threePoints(CS1T, CC1T, CE1T);
                collectSecondStack.twoPoints(CS2T, CE2T);
                collectSecondStack.threePoints(CS3TS, CC3TS, CE3TS, true, 0.5);

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

            while (!(phase == Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

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

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.009, 2);

                            if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                targetHeading = 310;
                            }

                            if (Math.abs(oneEightyHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingF.getY() - odometry.Y) < HeadingControlError){

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
                                    delivery.setMainPivot(delivery.getMainPivotPosition() + 0.005);

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
                    case fifth1:
                        delivery_and_collect_5();
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

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.008, 2);

                            if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < 30 && targetHeading == 270){

                                targetHeading = 320;

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
                    case fifth1:
                        delivery_and_collect_5();
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

            double p = 0.01;

            while (!(phase == Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow3Blue, odometry);

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

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, p);

                            if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                targetHeading = 355;
                                collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                collection.updateIntakeHeight();
                            }

                            if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && targetHeading == 355){

                                p = 0.025;

                                targetHeading = 265;

                                headingPos1.reset();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if(headingPos1.milliseconds() > 200 && targetHeading == 265){

                                p = 0.01;

                                targetHeading = 180;

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
                    case fifth1:
                        delivery_and_collect_5();
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
