package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue.Stage;

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
    Vector2D DPE1F = new Vector2D(getRealCoords(310), getRealCoords(78));

    /**delivery and collection points*/

    Vector2D DS1FC = new Vector2D(getRealCoords(46), getRealCoords(152));
    Vector2D DC1FC = new Vector2D(getRealCoords(50), getRealCoords(162));
    Vector2D DCC1FC = new Vector2D(getRealCoords(38), getRealCoords(160));
    Vector2D DE1FC = new Vector2D(getRealCoords(106), getRealCoords(152));

    Vector2D DS1F = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1F = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1F = new Vector2D(getRealCoords(106), getRealCoords(152));

    Vector2D DS2F = DE1F;
    Vector2D DE2F = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3F = DE2F;
    Vector2D DC3F = new Vector2D(getRealCoords(245), getRealCoords(157));
    Vector2D DE3F = new Vector2D(getRealCoords(315), getRealCoords(120));

    /**collecting paths*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(312), getRealCoords(156));
    Vector2D CE1F = new Vector2D(getRealCoords(220), getRealCoords(154));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(40), getRealCoords(175));
    Vector2D CE3F = new Vector2D(getRealCoords(40), getRealCoords(135));

    Vector2D CS3FS = CE2F;
    Vector2D CC3FS = new Vector2D(getRealCoords(52), getRealCoords(160));
    Vector2D CE3FS = new Vector2D(getRealCoords(42), getRealCoords(115));

    /**
     * second pos
     * */
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(71));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(62));
    Vector2D DPE1S = new Vector2D(getRealCoords(310), getRealCoords(90));

    /**delivery and collection points*/

    Vector2D DS1SC = new Vector2D(getRealCoords(40), getRealCoords(135));
    Vector2D DC1SC = new Vector2D(getRealCoords(44), getRealCoords(162));
    Vector2D DCC1SC = new Vector2D(getRealCoords(38), getRealCoords(160));
    Vector2D DE1SC = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 1 deliver straight
    Vector2D DS1S = new Vector2D(getRealCoords(46), getRealCoords(135));
    Vector2D DC1S = new Vector2D(getRealCoords(47), getRealCoords(151));
    Vector2D DE1S = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 2
    Vector2D DS2S = DE1S;
    Vector2D DE2S = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3S = DE2S;
    Vector2D DC3S = new Vector2D(getRealCoords(245), getRealCoords(157));
    Vector2D DE3S = new Vector2D(getRealCoords(315), getRealCoords(110));

    /**collecting paths*/
    Vector2D CS1S = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1S = new Vector2D(getRealCoords(312), getRealCoords(156));
    Vector2D CE1S = new Vector2D(getRealCoords(220), getRealCoords(154));

    Vector2D CS2S = CE1S;
    Vector2D CE2S = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3S = CE2S;
    Vector2D CC3S = new Vector2D(getRealCoords(40), getRealCoords(175));
    Vector2D CE3S = new Vector2D(getRealCoords(40), getRealCoords(135));

    Vector2D CS3SS = CE2S;
    Vector2D CC3SS = new Vector2D(getRealCoords(49), getRealCoords(170));
    Vector2D CE3SS = new Vector2D(getRealCoords(39), getRealCoords(115));

    /**
     * Third position
     * */

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(220), getRealCoords(79));
    Vector2D DPCT1T = new Vector2D(getRealCoords(160), getRealCoords(110));
    Vector2D DPE1T = new Vector2D(getRealCoords(310), getRealCoords(115));

    /**delivery and collection points*/
    //segment 1 recollect
    Vector2D DS1TC = new Vector2D(getRealCoords(42), getRealCoords(135));
    Vector2D DC1TC = new Vector2D(getRealCoords(52), getRealCoords(162));
    Vector2D DCC1TC = new Vector2D(getRealCoords(41), getRealCoords(160));
    Vector2D DE1TC = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 1 straight deliver
    Vector2D DS1T = new Vector2D(getRealCoords(42), getRealCoords(135));
    Vector2D DC1T = new Vector2D(getRealCoords(46), getRealCoords(151));
    Vector2D DE1T = new Vector2D(getRealCoords(106), getRealCoords(152));

    //segment 2
    Vector2D DS2T = DE1T;
    Vector2D DE2T = new Vector2D(getRealCoords(206), getRealCoords(154));

    //segment 3
    Vector2D DS3T = DE2T;
    Vector2D DC3T = new Vector2D(getRealCoords(245), getRealCoords(157));
    Vector2D DE3T = new Vector2D(getRealCoords(315), getRealCoords(110));

    /**collecting paths*/
    Vector2D CS1T = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1T = new Vector2D(getRealCoords(312), getRealCoords(156));
    Vector2D CE1T = new Vector2D(getRealCoords(220), getRealCoords(154));

    Vector2D CS2T = CE1T;
    Vector2D CE2T = new Vector2D(getRealCoords(91), getRealCoords(153));

    Vector2D CS3T = CE2T;
    Vector2D CC3T = new Vector2D(getRealCoords(40), getRealCoords(175));
    Vector2D CE3T = new Vector2D(getRealCoords(40), getRealCoords(135));

    /**Action points*/
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(180), getRealCoords(150));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(125), getRealCoords(150));
    Vector2D reverseIntake = new Vector2D(getRealCoords(72), getRealCoords(150));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(245), getRealCoords(80));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(220), getRealCoords(65));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(241), getRealCoords(86));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(218), getRealCoords(78));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(227), getRealCoords(105));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(210), getRealCoords(50));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(180), getRealCoords(180));
    Vector2D armOverPosition = new Vector2D(getRealCoords(190), getRealCoords(180));

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

    Blue_Close_Stage.Phase phase = Phase.preload;

    Blue_Close_Stage.Build build = Build.notBuilt;

    Blue_Close_Stage.Auto auto = Blue_Close_Stage.Auto.preload;

    Collection.intakePowerState previousState = Collection.intakePowerState.off;

    ElapsedTime buildPaths = new ElapsedTime();

    ElapsedTime reverseIntakeTimer = new ElapsedTime();

    Vector2D secondStack = new Vector2D(getRealCoords(41), getRealCoords(115));

    public void delivery_and_collect_2() throws InterruptedException {

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

        }

        if (collection.getIntakeCurrentUse() > 5500 && !reversingIntake){
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

        if (gripperControl.milliseconds() > (timeChanger+700) && gripperControl.milliseconds() < (timeChanger+800) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (gripperControl.milliseconds() > (timeChanger+800) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1200) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

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

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 10 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200 && odometry.X > 200){

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

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < IntakeControlError){

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

        }

        if (pathing){

            if (delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 3);
            }


        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError - 2){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.secondAndHalf);
            collection.updateIntakeHeight();

            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliverRecollect.followablePath, deliverRecollect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

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

                collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
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

        if (collection.getIntakeCurrentUse() > 5500 && !reversingIntake){
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

        if (gripperControl.milliseconds() > (timeChanger+700) && gripperControl.milliseconds() < (timeChanger+800) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (gripperControl.milliseconds() > (timeChanger+800) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1200) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

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

            if(pathing && Math.abs(odometry.getVerticalVelocity()) < 10 && !sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200&& odometry.X > 200){

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

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < IntakeControlError){

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
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 3);
            }


        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError - 2){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
            collection.updateIntakeHeight();

            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliverRecollect.followablePath, deliverRecollect.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

        }
    }

    public void delivery_and_collect_6() throws InterruptedException{

        odometry.update();

//        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());
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

                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
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

        if (collection.getIntakeCurrentUse() > 5500 && !reversingIntake){
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

        if (gripperControl.milliseconds() > (timeChanger+700) && gripperControl.milliseconds() < (timeChanger+800) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (gripperControl.milliseconds() > (timeChanger+800) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1200) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

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

            if (Math.abs(turnIntakeOff.getX() - odometry.X) < IntakeControlError && Math.abs(turnIntakeOff.getY() - odometry.Y) < IntakeControlError) {

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.off);
                collection.updateIntakeState();

            }

            if (Math.abs(reverseIntake.getX() - odometry.X) < IntakeControlError && Math.abs(reverseIntake.getY() - odometry.Y) < IntakeControlError){

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
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 3);
            }


        }else if (Math.abs(secondStack.getX() - odometry.X) < collectionError && Math.abs(secondStack.getY() - odometry.Y) < collectionError - 2){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
            collection.updateIntakeHeight();

            pathing = true;

            armOver = false;

            delivering = true;

            follower.setPath(deliverRecollect.followablePath, deliverRecollect.pathingVelocity);

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
                collect.threePoints(CS3F, CC3F, CE3F, true, 0.5);

                collectSecondStack.threePoints(CS1F, CC1F, CE1F);
                collectSecondStack.twoPoints(CS2F, CE2F);
                collectSecondStack.threePoints(CS3FS, CC3FS, CE3FS, true, 0.5);

                deliver.threePoints(DS1F, DC1F, DE1F);
                deliver.twoPoints(DS2F, DE2F);
                deliver.threePoints(DS3F, DC3F, DE3F, true, 0.7);

                deliverRecollect.fourPoints(DS1FC, DC1FC, DCC1FC, DE1FC);
                deliverRecollect.twoPoints(DS2F, DE2F);
                deliverRecollect.threePoints(DS3F, DC3F, DE3F, true, 0.7);

                DeliveryEndpoint = DE3F;
                CollectionEndpoint = CE3F;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 2){

                clearAll();

                preloadPaths.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true);

                collect.threePoints(CS1S, CC1S, CE1S);
                collect.twoPoints(CS2S, CE2S);
                collect.threePoints(CS3S, CC3S, CE3S, true, 0.5);

                collectSecondStack.threePoints(CS1S, CC1S, CE1S);
                collectSecondStack.twoPoints(CS2S, CE2S);
                collectSecondStack.threePoints(CS3SS, CC3SS, CE3SS, true, 0.5);

                deliver.threePoints(DS1S, DC1S, DE1S);
                deliver.twoPoints(DS2S, DE2S);
                deliver.threePoints(DS3S, DC3S, DE3S, true, 0.7);

                deliverRecollect.fourPoints(DS1SC, DC1SC, DCC1SC, DE1SC);
                deliverRecollect.twoPoints(DS2S, DE2S);
                deliverRecollect.threePoints(DS3S, DC3S, DE3S, true, 0.7);

                DeliveryEndpoint = DE3S;
                CollectionEndpoint = CE3S;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 3){

                clearAll();

                preloadPaths.fourPoints(DPS1T, DPC1T, DPCT1T, DPE1T, true, 0.45);

                collect.threePoints(CS1T, CC1T, CE1T);
                collect.twoPoints(CS2T, CE2T);
                collect.threePoints(CS3T, CC3T, CE3T, true, 0.5);

                collectSecondStack.threePoints(CS1T, CC1T, CE1T);
                collectSecondStack.twoPoints(CS2T, CE2T);
                collectSecondStack.threePoints(CS3T, CC3T, new Vector2D(CE3T.getX(), (CE3T.getY()-10)), true, 0.5);

                deliver.threePoints(DS1T, DC1T, DE1T);
                deliver.twoPoints(DS2T, DE2T);
                deliver.threePoints(DS3T, DC3T, DE3T, true, 0.7);

                deliverRecollect.fourPoints(DS1TC, DC1TC, DCC1TC, DE1TC);
                deliverRecollect.twoPoints(DS2T, DE2T);
                deliverRecollect.threePoints(DS3T, DC3T, DE3T, true, 0.7);

                DeliveryEndpoint = DE3T;
                CollectionEndpoint = CE3T;

                buildPaths.reset();
            }

        }

        waitForStart();

        if (propPos == 1){

            while (!(phase == Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                        collection.updateIntakeHeight();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.12, 2);

                            if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                targetHeading = 290;

                                collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                collection.updateIntakeHeight();
                            }

                            if (Math.abs(oneEightyHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingF.getY() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                            }

                            if (deliverySlides.getCurrentposition() > 200 && sensors.armSensor.isPressed()){

                                drive.setAllPower(0);

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

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

                        }else if (Math.abs(DPE1F.getX() - odometry.X) < deliveryError && Math.abs(DPE1F.getY() - odometry.Y) < deliveryError && !pathing){

                            drive.setAllPower(0);

                            if (sensors.armSensor.isPressed()){

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

                            }else {
                                delivery.ArmExtension.setPosition(0.8);

                                sleep(400);

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

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.01, 2);

                            if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < 10 && targetHeading == 270){
                                targetHeading = 330;

                                collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                collection.updateIntakeHeight();
                            }

                            if (Math.abs(oneEightyHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingS.getY() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (deliverySlides.getCurrentposition() > 200 && sensors.armSensor.isPressed()){

                                drive.setAllPower(0);

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

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

                            if (deliverySlides.getCurrentposition() > 150 && deliverySlides.getCurrentposition() < 300){

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
                                delivery.ArmExtension.setPosition(0.8);

                                sleep(400);

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

        }else if (propPos == 3) {

            while (!(phase == Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();
                        delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow3Blue, odometry);

                        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                        collection.updateIntakeHeight();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.015);

                            if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                targetHeading = 350;
                                collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                collection.updateIntakeHeight();
                            }

                            if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingT.getY() - odometry.Y) < HeadingControlError){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (deliverySlides.getCurrentposition() > 200 && sensors.armSensor.isPressed()){

                                drive.setAllPower(0);

                                sleep(200);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(200);

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

                            if (deliverySlides.getCurrentposition() > 150 && deliverySlides.getCurrentposition() < 300){

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
                                delivery.ArmExtension.setPosition(0.8);

                                sleep(400);

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
