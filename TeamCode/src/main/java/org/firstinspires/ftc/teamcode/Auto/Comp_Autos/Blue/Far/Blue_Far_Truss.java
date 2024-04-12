package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue.Far;

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

@Autonomous(name = "Blue_Far_Truss", group = "blue auto's")
public class Blue_Far_Truss extends LinearOpMode implements CycleMethods {

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
    Vector2D DPE1F = new Vector2D(getRealCoords(43), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1F = DPE1F;
    Vector2D DYC1F = new Vector2D(getRealCoords(40), getRealCoords(68));
    Vector2D DYE1F = new Vector2D(getRealCoords(90), getRealCoords(39));

    Vector2D DYS2F = DYE1F;
    Vector2D DYE2F = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3F = DYE2F;
    Vector2D DYC3F = new Vector2D(getRealCoords(261), getRealCoords(31));
    Vector2D DYE3F = new Vector2D(getRealCoords(314), getRealCoords(76));

    /**delivery and collection points*/

    Vector2D DS1F = new Vector2D(getRealCoords(44), getRealCoords(90));
    Vector2D DC1F = new Vector2D(getRealCoords(53), getRealCoords(31));
    Vector2D DE1F = new Vector2D(getRealCoords(119), getRealCoords(36));

    Vector2D DS2F = DE1F;
    Vector2D DE2F = new Vector2D(getRealCoords(181), getRealCoords(33));

    //segment 3
    Vector2D DS3F = DE2F;
    Vector2D DC3F = new Vector2D(getRealCoords(220), getRealCoords(55));
    Vector2D DE3F = new Vector2D(getRealCoords(320), getRealCoords(70));

    /**collecting paths*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(265), getRealCoords(36));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(38));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(119), getRealCoords(36));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(63), getRealCoords(35));
    Vector2D CE3F = new Vector2D(getRealCoords(44), getRealCoords(105));

    /**Middle Prop position*/
    //purple pixel
    Vector2D DPS1S = startPosition;
    Vector2D DPC1S = new Vector2D(getRealCoords(104), getRealCoords(129));
    Vector2D DPCT1S = new Vector2D(getRealCoords(66), getRealCoords(83));
    Vector2D DPE1S = new Vector2D(getRealCoords(44), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1S = DPE1S;
    Vector2D DYC1S = new Vector2D(getRealCoords(40), getRealCoords(68));
    Vector2D DYE1S = new Vector2D(getRealCoords(90), getRealCoords(39));

    Vector2D DYS2S = DYE1S;
    Vector2D DYE2S = new Vector2D(getRealCoords(210), getRealCoords(35));

    Vector2D DYS3S = DYE2S;
    Vector2D DYC3S = new Vector2D(getRealCoords(261), getRealCoords(34));
    Vector2D DYE3S = new Vector2D(getRealCoords(316), getRealCoords(89));

    /**delivery and collection points*/

    Vector2D DS1S = new Vector2D(getRealCoords(44), getRealCoords(90));
    Vector2D DC1S = new Vector2D(getRealCoords(53), getRealCoords(31));
    Vector2D DE1S = new Vector2D(getRealCoords(119), getRealCoords(36));

    Vector2D DS2S = DE1S;
    Vector2D DE2S = new Vector2D(getRealCoords(181), getRealCoords(33));

    //segment 3
    Vector2D DS3S = DE2S;
    Vector2D DC3S = new Vector2D(getRealCoords(220), getRealCoords(50));
    Vector2D DE3S = new Vector2D(getRealCoords(320), getRealCoords(65));

    /**collecting paths*/
    Vector2D CS1S = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1S = new Vector2D(getRealCoords(265), getRealCoords(38));
    Vector2D CE1S = new Vector2D(getRealCoords(180), getRealCoords(36));

    Vector2D CS2S = CE1S;
    Vector2D CE2S = new Vector2D(getRealCoords(119), getRealCoords(32));

    Vector2D CS3S = CE2S;
    Vector2D CC3S = new Vector2D(getRealCoords(35), getRealCoords(75));
    Vector2D CE3S = new Vector2D(getRealCoords(44), getRealCoords(105));

    /**Right Prop position*/

    //purple pixel
    Vector2D DPS1T = startPosition;
    Vector2D DPE1T = new Vector2D(getRealCoords(74), getRealCoords(85));

    Vector2D DPS2T = DPE1T;
    Vector2D DPC2T = new Vector2D(getRealCoords(70), getRealCoords(60));
    Vector2D DPE2T = new Vector2D(getRealCoords(48), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1T = DPE1T;
    Vector2D DYC1T = new Vector2D(getRealCoords(40), getRealCoords(68));
    Vector2D DYE1T = new Vector2D(getRealCoords(90), getRealCoords(39));

    Vector2D DYS2T = DYE1T;
    Vector2D DYE2T = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3T = DYE2T;
    Vector2D DYC3T = new Vector2D(getRealCoords(261), getRealCoords(31));
    Vector2D DYE3T = new Vector2D(getRealCoords(318), getRealCoords(104));

    /**delivery and collection points*/

    Vector2D DS1T = new Vector2D(getRealCoords(46), getRealCoords(90));
    Vector2D DC1T = new Vector2D(getRealCoords(55), getRealCoords(34));
    Vector2D DE1T = new Vector2D(getRealCoords(119), getRealCoords(35));

    Vector2D DS2T = DE1T;
    Vector2D DE2T = new Vector2D(getRealCoords(181), getRealCoords(33));

    //segment 3
    Vector2D DS3T = DE2T;
    Vector2D DC3T = new Vector2D(getRealCoords(220), getRealCoords(54));
    Vector2D DE3T = new Vector2D(getRealCoords(320), getRealCoords(66));

    /**collecting paths*/
    Vector2D CS1T = new Vector2D(getRealCoords(300), getRealCoords(95));
    Vector2D CC1T = new Vector2D(getRealCoords(265), getRealCoords(41));
    Vector2D CE1T = new Vector2D(getRealCoords(180), getRealCoords(38));

    Vector2D CS2T = CE1T;
    Vector2D CE2T = new Vector2D(getRealCoords(119), getRealCoords(34));

    Vector2D CS3T = CE2T;
    Vector2D CC3T = new Vector2D(getRealCoords(63), getRealCoords(35));
    Vector2D CE3T = new Vector2D(getRealCoords(45), getRealCoords(95));

    /**Action points*/
    Vector2D extendSlides = new Vector2D(getRealCoords(224), getRealCoords(32));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(90), getRealCoords(70));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(90), getRealCoords(55));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(77), getRealCoords(91));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(86), getRealCoords(72));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(80), getRealCoords(70));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(85), getRealCoords(38));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(150), getRealCoords(32));
    Vector2D deployArm = new Vector2D(getRealCoords(180), getRealCoords(32));

    //collection
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(183), getRealCoords(32));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(125), getRealCoords(32));
    Vector2D restartIntake = new Vector2D(getRealCoords(75), getRealCoords(32));
    Vector2D reverseIntake = new Vector2D(getRealCoords(49), getRealCoords(60));

    /**path objects*/
    GenMethods purple = new GenMethods();
    GenMethods thirdPosSecond = new GenMethods();
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

    int slidesPosYellowPixel = 300;

    /**hardware objects*/
    Odometry odometry = new Odometry(startPosition.getX(), startPosition.getY(), 270);

    Drivetrain drive = new Drivetrain();

    mecanumFollower follower = new mecanumFollower();

    ElapsedTime gripperControl = new ElapsedTime();

    public void clearAll(){
        yellow.ClearAll();
        purple.ClearAll();
        thirdPosSecond.ClearAll();
        deliver.ClearAll();
        collect.ClearAll();
    }

    /**booleans*/
    boolean lockIn = false;

    boolean timerSet = false;

    boolean pathing = false;

    boolean delivering = false;

    boolean gotTwo = false;

    boolean onlyOnce = true;

    /**doubles*/
    long waitYellow = 0;

    double targetHeading = 0;

    double timeChanger;

    boolean armOver = false;

    double intakeCurrentOne = 5500;
    double intakeNormal = 5500;

    boolean reversingIntake = false;
    Collection.intakePowerState previousState = Collection.intakePowerState.off;
    ElapsedTime reverseIntakeTimer = new ElapsedTime();

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

        collection.setIntakeHeight(Collection.intakeHeightState.forthAndHalf);
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

//        if (counter < 40){
//            collection.setState(Collection.intakePowerState.reversed);
//            collection.updateIntakeState();
//
//            sleep(100);
//
//            collection.setState(Collection.intakePowerState.on);
//            collection.updateIntakeState();
//        }

        while (counter < 40){
            counter++;
            sleep(50);

            collection.IntakeHeightRight.setPosition(collection.getIntakeHeightRight() - 0.005);

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
            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
            if (gotTwo){
                counter = 45;
            }
        }


        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

        sleep(200);

        delivery.ArmExtension.setPosition(1);

        collection.setState(Collection.intakePowerState.reversed);
        collection.updateIntakeState();

        sleep(200);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

    }

    public void delivery_and_collect_2() throws InterruptedException{

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

        if (build == Build.notBuilt) {
            follower.setPath(collect.followablePath, collect.pathingVelocity);
            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
        }

        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Build.notBuilt){

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

                collection.setIntakeHeight(Collection.intakeHeightState.secondAndHalf);
                collection.updateIntakeHeight();

                delivery.ArmExtension.setPosition(delivery.ArmExtensionHome);
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

        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

            gripperControl.reset();

            drive.RF.setPower(0);
            drive.RB.setPower(0);
            drive.LF.setPower(0);
            drive.LB.setPower(0);

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            delivering = true;

            onlyOnce = false;

            gotTwo = true;

        }

        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+400) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (gripperControl.milliseconds() > (timeChanger+400) && gripperControl.milliseconds() < (timeChanger+600) && gotTwo){

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+600) && gripperControl.milliseconds() < (timeChanger+800) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() < -10) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > deployArm.getX() && deliverySlides.getCurrentposition() > 200 && odometry.getVerticalVelocity() < -10 && !armOver){

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                armOver = true;

            }

            if (sensors.armSensor.isPressed() && deliverySlides.getCurrentposition() > 200){

                if (auto == Blue_Far_Truss.Auto.two) {
                    drive.setAllPower(0.4);
                }else{
                    drive.setAllPower(0.2);
                }

                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Blue_Far_Truss.Auto.two) {

                    phase = Blue_Far_Truss.Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Blue_Far_Truss.Phase.second2;

                }

            }

            if (pathing && Math.abs(odometry.getVerticalVelocity()) < 10 && !sensors.armSensor.isPressed() && odometry.X > 200 && deliverySlides.getCurrentposition() > 200){

                if (auto == Blue_Far_Truss.Auto.two) {
                    drive.setAllPower(0.4);
                }else{
                    drive.setAllPower(0.2);
                }

                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                gotTwo = false;

                armOver = false;

                if (auto == Blue_Far_Truss.Auto.two) {

                    phase = Blue_Far_Truss.Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Blue_Far_Truss.Phase.second2;

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

            if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < 20){

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

            }

            if (Math.abs(DeliveryEndpoint.getX() - odometry.X) < deliveryError && Math.abs(DeliveryEndpoint.getY() - odometry.Y) < deliveryError && !pathing) {

                drive.setAllPower(0.4);

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

                if (auto == Blue_Far_Truss.Auto.two) {

                    phase = Blue_Far_Truss.Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Blue_Far_Truss.Phase.second2;

                }
            }

        }

        if (pathing){

            if(delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
            }

        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError && !pathing){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
            collection.updateIntakeHeight();

            if (sensors.RightClawSensor.isPressed() || sensors.LeftClawSensor.isPressed()){
                drive.strafeLeft();

                sleep(300);

                drive.strafeRight();

                sleep(300);

                drive.setAllPower(0);
            }

            pathing = true;

            onlyOnce = false;

            delivering = true;

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

        }
    }

    public void delivery_and_collect_4() throws InterruptedException{

        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

        if (build == Build.notBuilt) {
            follower.setPath(collect.followablePath, collect.pathingVelocity);
            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
        }

        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (build == Build.notBuilt){

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

                collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
                collection.updateIntakeHeight();

                delivery.ArmExtension.setPosition(delivery.ArmExtensionHome);
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

        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

            gripperControl.reset();

            drive.setAllPower(0);

            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

            delivering = true;

            onlyOnce = false;

            gotTwo = true;

        }

        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+400) && gotTwo){

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (gripperControl.milliseconds() > (timeChanger+400) && gripperControl.milliseconds() < (timeChanger+600) && gotTwo){

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+600) && gripperControl.milliseconds() < (timeChanger+800) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX() && odometry.getVerticalVelocity() < -10) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (odometry.X > deployArm.getX() && deliverySlides.getCurrentposition() > 200 && odometry.getVerticalVelocity() < -10 && !armOver){

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

                phase = Blue_Far_Truss.Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                }

            }

            if (Math.abs(odometry.getVerticalVelocity()) < 10 && !sensors.armSensor.isPressed() && odometry.X > 180 && deliverySlides.getCurrentposition() > 200){

                drive.setAllPower(0.4);

                delivery.setGripperState(Delivery.GripperState.open);

                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                phase = Blue_Far_Truss.Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
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

            if (Math.abs(restartIntake.getX() - odometry.X) < IntakeControlError && Math.abs(restartIntake.getY() - odometry.Y) < 20){

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                collection.setState(Collection.intakePowerState.on);
                collection.updateIntakeState();

            }

            if (Math.abs(DeliveryEndpoint.getX() - odometry.X) < deliveryError && Math.abs(DeliveryEndpoint.getY() - odometry.Y) < deliveryError && !pathing) {

                drive.setAllPower(0.4);

                drive.RF.setPower(0);
                drive.RB.setPower(0);
                drive.LF.setPower(0);
                drive.LB.setPower(0);

                delivery.setGripperState(Delivery.GripperState.open);
                delivery.updateGrippers();

                sleep(200);

                delivery.setArmTargetState(Delivery.armState.collect);
                delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);

                deliverySlides.DeliverySlides(0, -0.5);

                pathing = true;

                phase = Blue_Far_Truss.Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                }

            }

        }

        if (pathing){

            pathing = follower.followPathAuto(targetHeading, odometry, drive);

        }else if (Math.abs(CollectionEndpoint.getX() - odometry.X) < collectionError && Math.abs(CollectionEndpoint.getY() - odometry.Y) < collectionError && !pathing){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
            collection.updateIntakeHeight();

            if (sensors.RightClawSensor.isPressed() || sensors.LeftClawSensor.isPressed()){
                drive.strafeLeft();

                sleep(300);

                drive.strafeRight();

                sleep(300);

                drive.setAllPower(0);
            }

            pathing = true;

            onlyOnce = false;

            delivering = true;

            armOver = false;

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

    Blue_Far_Truss.Phase phase = Phase.preload;

    Blue_Far_Truss.Build build = Build.notBuilt;

    Blue_Far_Truss.Auto auto = Blue_Far_Truss.Auto.preload;

    Preload preload = Preload.purple;

    ElapsedTime buildPaths = new ElapsedTime();

    ElapsedTime extendSlidesPreload = new ElapsedTime();

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        buildPaths.reset();

        while (!timerSet){
            telemetry.addData("time to wait for yellow preload", waitYellow);
            telemetry.addData("press d pad up to increase", "");
            telemetry.addData("press d pad down to decrease", "");
            telemetry.addData("press x to lock in!!!!", "");
            telemetry.update();

            if (gamepad1.dpad_down){
                waitYellow -= 50;
            } else if (gamepad1.dpad_up) {
                waitYellow += 50;
            }else if (gamepad1.x) {
                timerSet = true;
            }

        }

        while(!lockIn){

            telemetry.addData("Auto activated", auto);
            telemetry.addData("press y for 2+1", "");
            telemetry.addData("press a for 2+3", "");
            telemetry.addData("press b for 2+5", "");
            telemetry.addData("press x to lock in!!!!", "");
            telemetry.update();

            if (gamepad1.a){
                auto = Blue_Far_Truss.Auto.two;
            } else if (gamepad1.b) {
                auto = Blue_Far_Truss.Auto.four;
            } else if (gamepad1.y) {
                auto = Blue_Far_Truss.Auto.preload;
            }else if (gamepad1.x) {
                lockIn = true;
            }

        }

        while(opModeInInit()){

            if (buildPaths.seconds() > 2 && propPos == 1){

                clearAll();

                purple.fourPoints(DPS1F, DPC1F, DPCT1F, DPE1F, true, 0.65);

                yellow.threePoints(DYS1F, DYC1F, DYE1F);
                yellow.twoPoints(DYS2F, DYE2F);
                yellow.threePoints(DYS3F, DYC3F, DYE3F, true, 1);

                collect.threePoints(CS1S, CC1S, CE1S);
                collect.twoPoints(CS2S, CE2S);
                collect.threePoints(CS3S, CC3S, CE3S, true, 0.4);

                deliver.threePoints(DS1S, DC1S, DE1S);
                deliver.twoPoints(DS2S, DE2S);
                deliver.threePoints(DS3S, DC3S, DE3S, true);

                DeliveryEndpoint = DE3S;
                CollectionEndpoint = CE3S;

                buildPaths.reset();
            } else if (buildPaths.seconds() > 2 && propPos == 2){

                clearAll();

                purple.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true, 0.8);

                //Build yellow preload path with three segments
                yellow.threePoints(DYS1S, DYC1S, DYE1S);
                yellow.twoPoints(DYS2S, DYE2S);
                yellow.threePoints(DYS3S, DYC3S, DYE3S, true, 1);

                collect.threePoints(CS1S, CC1S, CE1S);
                collect.twoPoints(CS2S, CE2S);
                collect.threePoints(CS3S, CC3S, CE3S, true, 0.4);

                deliver.threePoints(DS1S, DC1S, DE1S);
                deliver.twoPoints(DS2S, DE2S);
                deliver.threePoints(DS3S, DC3S, DE3S, true);

                DeliveryEndpoint = DE3S;
                CollectionEndpoint = CE3S;

                buildPaths.reset();

            } else if (buildPaths.seconds() > 2 && propPos == 3){

                clearAll();

                purple.twoPoints(DPS1T, DPE1T, true, 1.4);
                thirdPosSecond.threePoints(DPS2T, DPC2T, DPE2T, true, 1);

                yellow.threePoints(DYS1T, DYC1T, DYE1T);
                yellow.twoPoints(DYS2T, DYE2T);
                yellow.threePoints(DYS3T, DYC3T, DYE3T, true, 1.1);

                collect.threePoints(CS1T, CC1T, CE1T);
                collect.twoPoints(CS2T, CE2T);
                collect.threePoints(CS3T, CC3T, CE3T, true, 0.4);

                deliver.threePoints(DS1T, DC1T, DE1T);
                deliver.twoPoints(DS2T, DE2T);
                deliver.threePoints(DS3T, DC3T, DE3T, true);

                DeliveryEndpoint = DE3T;
                CollectionEndpoint = CE3T;

                buildPaths.reset();
            }

        }

        waitForStart();

        if (propPos == 1){

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Far_Truss.Phase.finished)){

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

                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.015);

                                    if (Math.abs(leavePurpleHeadingF.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingF.getY() - odometry.Y) < HeadingControlError && deliverySlides.getCurrentposition() < 50){
                                        targetHeading = 180;

                                        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                        collection.updateIntakeHeight();
                                    }

                                }else if (Math.abs(DPE1F.getX() - odometry.X) < deliveryError && Math.abs(DPE1F.getY() - odometry.Y) < deliveryError && !pathing){

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    collectOnePixel();

                                    build = Build.notBuilt;

                                    sleep(waitYellow);

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

                                }else if (Math.abs(DYE3F.getX() - odometry.X) < deliveryError && Math.abs(DYE3F.getY() - odometry.Y) < deliveryError && !pathing){

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

                                        delivery.setLeftGripperState(Delivery.leftGripperState.open);
                                        delivery.updateGrippers();

                                        sleep(200);

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

                                        phase = Blue_Far_Truss.Phase.finished;

                                    }else {

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow1Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                        phase = Blue_Far_Truss.Phase.first2;

                                        build = Blue_Far_Truss.Build.notBuilt;

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

        } else if (propPos == 2) {

            // while loop to keep auto running while tasks are not complete
            while (!(phase == Blue_Far_Truss.Phase.finished)){

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
                                    targetHeading = 270;

                                    //make sure yellow pixel is secure
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();

                                }

                                if (pathing){

                                    //pathing method, pass in odometry and drivetrain objects as parameters
                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.012);

                                    //turn to leave purple pixel on spike mark
                                    if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                        targetHeading = 200;
                                        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                        collection.updateIntakeHeight();
                                    }

                                    //straighten robot to collect from stack
                                    if (Math.abs(DPE1S.getX() - odometry.X) < HeadingControlError+15 && Math.abs(DPE1S.getY() - odometry.Y) < HeadingControlError+15){
                                        targetHeading = 180;
                                    }

                                }else if (Math.abs(DPE1S.getX() - odometry.X) < collectionError && Math.abs(DPE1S.getY() - odometry.Y) < collectionError && !pathing){

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    collectOnePixel();

                                    build = Build.notBuilt;

                                    sleep(waitYellow);

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

                                }else if (Math.abs(DYE3S.getX() - odometry.X) < deliveryError && Math.abs(DYE3S.getY() - odometry.Y) < deliveryError && !pathing){


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

                                        phase = Blue_Far_Truss.Phase.finished;

                                    }else {

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                        phase = Blue_Far_Truss.Phase.first2;

                                        build = Blue_Far_Truss.Build.notBuilt;

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

        }else if (propPos == 3) {

            while (!(phase == Blue_Far_Truss.Phase.finished)){

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

                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.015);

                                    if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                        targetHeading = 300;
                                    }

                                    if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 305){
                                        targetHeading = 180;
                                        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                                        collection.updateIntakeHeight();
                                    }

                                } else if (Math.abs(DPE1T.getX() - odometry.X) < collectionError && Math.abs(DPE1T.getY() - odometry.Y) < collectionError && !pathing){

                                    follower.setPath(thirdPosSecond.followablePath, thirdPosSecond.pathingVelocity);

                                    pathing = true;

                                    targetHeading = 305;

                                } else if (Math.abs(DPE2T.getX() - odometry.X) < collectionError && Math.abs(DPE2T.getY() - odometry.Y) < collectionError && !pathing){

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    collectOnePixel();

                                    build = Build.notBuilt;

                                    sleep(waitYellow);

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
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

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
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        while (deliverySlides.getCurrentposition() > 20){
                                            delivery.updateArm(deliverySlides.getCurrentposition(), false,  Delivery.PixelsAuto.yellow3Blue, odometry);
                                        }

                                        phase = Blue_Far_Truss.Phase.finished;

                                    }else {

                                        delivery.setArmTargetState(Delivery.armState.collect);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow3Blue, odometry);

                                        deliverySlides.DeliverySlides(0, -0.5);

                                        deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                        phase = Blue_Far_Truss.Phase.first2;

                                        build = Blue_Far_Truss.Build.notBuilt;

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
