package org.firstinspires.ftc.teamcode.Auto.Comp_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
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
    Vector2D DPE1F = new Vector2D(getRealCoords(44), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1F = DPE1F;
    Vector2D DYC1F = new Vector2D(getRealCoords(40), getRealCoords(68));
    Vector2D DYE1F = new Vector2D(getRealCoords(90), getRealCoords(39));

    Vector2D DYS2F = DYE1F;
    Vector2D DYE2F = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3F = DYE2F;
    Vector2D DYC3F = new Vector2D(getRealCoords(261), getRealCoords(31));
    Vector2D DYE3F = new Vector2D(getRealCoords(305), getRealCoords(85));

    /**Middle Prop position*/
    //purple pixel
    Vector2D DPS1S = startPosition;
    Vector2D DPC1S = new Vector2D(getRealCoords(104), getRealCoords(129));
    Vector2D DPCT1S = new Vector2D(getRealCoords(66), getRealCoords(83));
    Vector2D DPE1S = new Vector2D(getRealCoords(45), getRealCoords(93));

    //yellow pixel
    Vector2D DYS1S = DPE1S;
    Vector2D DYC1S = new Vector2D(getRealCoords(40), getRealCoords(68));
    Vector2D DYE1S = new Vector2D(getRealCoords(90), getRealCoords(39));

    Vector2D DYS2S = DYE1S;
    Vector2D DYE2S = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3S = DYE2S;
    Vector2D DYC3S = new Vector2D(getRealCoords(261), getRealCoords(31));
    Vector2D DYE3S = new Vector2D(getRealCoords(305), getRealCoords(89));

    /**Right Prop position*/

    //purple pixel
    Vector2D DPS1T = startPosition;
    Vector2D DPE1T = new Vector2D(getRealCoords(75), getRealCoords(93));

    Vector2D DPS2T = DPE1T;
    Vector2D DPC2T = new Vector2D(getRealCoords(70), getRealCoords(60));
    Vector2D DPE2T = new Vector2D(getRealCoords(44), getRealCoords(90));

    //yellow pixel
    Vector2D DYS1T = DPE1T;
    Vector2D DYC1T = new Vector2D(getRealCoords(40), getRealCoords(68));
    Vector2D DYE1T = new Vector2D(getRealCoords(90), getRealCoords(39));

    Vector2D DYS2T = DYE1T;
    Vector2D DYE2T = new Vector2D(getRealCoords(210), getRealCoords(32));

    Vector2D DYS3T = DYE2T;
    Vector2D DYC3T = new Vector2D(getRealCoords(261), getRealCoords(31));
    Vector2D DYE3T = new Vector2D(getRealCoords(305), getRealCoords(110));

    /**delivery and collection points*/

    Vector2D DS1F = new Vector2D(getRealCoords(44), getRealCoords(90));

    Vector2D DC1F = new Vector2D(getRealCoords(53), getRealCoords(31));
    Vector2D DE1F = new Vector2D(getRealCoords(119), getRealCoords(36));

    Vector2D DS2F = DE1F;

    Vector2D DE2F = new Vector2D(getRealCoords(181), getRealCoords(33));

    //segment 3
    Vector2D DS3F = DE2F;

    Vector2D DC3F = new Vector2D(getRealCoords(231), getRealCoords(55));

    Vector2D DCC3F = new Vector2D(getRealCoords(292), getRealCoords(29));

    Vector2D DE3F = new Vector2D(getRealCoords(320), getRealCoords(65));

    /**collecting paths*/
    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(265), getRealCoords(31));
    Vector2D CE1F = new Vector2D(getRealCoords(180), getRealCoords(33));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(119), getRealCoords(34));

    Vector2D CS3F = CE2F;
    Vector2D CC3F = new Vector2D(getRealCoords(72), getRealCoords(30));
    Vector2D CE3F = new Vector2D(getRealCoords(74), getRealCoords(44));

    Vector2D CS4F = CE3F;
    Vector2D CC4F = new Vector2D(getRealCoords(70), getRealCoords(78));
    Vector2D CCC4F = new Vector2D(getRealCoords(39), getRealCoords(49));
    Vector2D CE4F = new Vector2D(getRealCoords(44), getRealCoords(110));

    /**Action points*/

    //preload
    Vector2D extendSlides = new Vector2D(getRealCoords(224), getRealCoords(32));

    //first position
    Vector2D oneEightyHeadingF = new Vector2D(getRealCoords(90), getRealCoords(70));
    Vector2D leavePurpleHeadingF = new Vector2D(getRealCoords(90), getRealCoords(65));

    //second position
    Vector2D oneEightyHeadingS = new Vector2D(getRealCoords(77), getRealCoords(91));
    Vector2D leavePurpleHeadingS = new Vector2D(getRealCoords(82), getRealCoords(76));

    //third position
    Vector2D oneEightyHeadingT = new Vector2D(getRealCoords(80), getRealCoords(70));
    Vector2D leavePurpleHeadingT = new Vector2D(getRealCoords(85), getRealCoords(38));

    //delivery
    Vector2D extendSlidesDelivery = new Vector2D(getRealCoords(140), getRealCoords(32));

    //collection
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(183), getRealCoords(32));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(125), getRealCoords(32));
    Vector2D reverseIntake = new Vector2D(getRealCoords(72), getRealCoords(33));

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

    int slidesPosYellowPixel = 400;

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

    boolean armOver = false;

    boolean reversingIntake = false;
    Collection.intakePowerState previousState = Collection.intakePowerState.off;
    ElapsedTime reverseIntakeTimer = new ElapsedTime();

    /**collection Methods*/
    public void collectPixels(boolean FirstPass){

//        delivery.setGripperState(Delivery.GripperState.open);
//        delivery.updateGrippers();
//
//        collection.setState(Collection.intakePowerState.on);
//        collection.updateIntakeState();
//
//        sleep(400);
//
//        collection.setIntakeHeight(Collection.intakeHeightState.collect);
//        collection.updateIntakeHeight();
//
//        sleep(2000);
//
//        delivery.setGripperState(Delivery.GripperState.closed);
//        delivery.updateGrippers();
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

        boolean gotTwo;

        int counter = 0;

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        if (FirstPass){
            collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
            collection.updateIntakeHeight();
        }else {
            collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
            collection.updateIntakeHeight();
        }

        while (counter < 14){
            counter++;
            sleep(100);
            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
            if (gotTwo){
                counter = 20;
            }
        }

        gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

        if (gotTwo){
            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();
        }else {

            if (FirstPass){
                collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                collection.updateIntakeHeight();
            }else {
                collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
                collection.updateIntakeHeight();
            }

            counter = 0;

            while (counter < 14){
                counter++;
                sleep(100);
                gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
                if (gotTwo){
                    counter = 20;
                }
            }

            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

            if (gotTwo){
                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();
            }else {

                counter = 0;

                while (counter < 14){
                    counter++;
                    sleep(100);
                    gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();
                    if (gotTwo){
                        counter = 20;
                    }
                }

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();
            }

        }

        delivery.ArmExtension.setPosition(1);

        sleep(100);

        collection.setState(Collection.intakePowerState.reversed);
        collection.updateIntakeState();

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

        sleep(300);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();

        delivery.setGripperState(Delivery.GripperState.closed);
        delivery.updateGrippers();

    }

    public void collectOnePixel(){

        int counter;

        delivery.setRightGripperState(Delivery.rightGripperState.open);
        delivery.updateGrippers();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        sleep(300);

        collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
        collection.updateIntakeHeight();

        counter = 0;

        while (counter < 20){
            counter++;
            sleep(50);
            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

            if (collection.getIntakeCurrentUse() > 4000 && !reversingIntake){
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
            if (collection.getIntakeCurrentUse() > 4000 && !reversingIntake){
                reversingIntake = true;
                reverseIntakeTimer.reset();
                previousState = collection.getPowerState();
                collection.setState(Collection.intakePowerState.reversed);
            }

            if (reversingIntake && reverseIntakeTimer.milliseconds() > 100){
                collection.setState(previousState);
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

    public void delivery_and_collect_2() throws InterruptedException {

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

                collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                collection.updateIntakeHeight();

                delivery.ArmExtension.setPosition(delivery.ArmExtensionHome);
            }

        }

        if (collection.getIntakeCurrentUse() > 4000 && !reversingIntake){
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

        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+400) && gotTwo){

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+400) && gripperControl.milliseconds() < (timeChanger+600) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX()) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (deliverySlides.getCurrentposition() > 200 && !armOver){

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

                if (auto == Blue_Auto_Right.Auto.two) {

                    phase = Blue_Auto_Right.Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Blue_Auto_Right.Phase.second2;

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

                collection.setState(Collection.intakePowerState.reversedHalf);
                collection.updateIntakeState();

            }

            if (Math.abs(DE3F.getX() - odometry.X) < deliveryError && Math.abs(DE3F.getY() - odometry.Y) < deliveryError && !pathing) {

                drive.setAllPower(-0.4);

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

                if (auto == Blue_Auto_Right.Auto.two) {

                    phase = Blue_Auto_Right.Phase.finished;

                    drive.setAllPower(0);

                    while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                    }

                } else {

                    build = Build.notBuilt;

                    delivering = false;

                    phase = Blue_Auto_Right.Phase.second2;

                }
            }

        }

        if (pathing){

            if(delivering){
                pathing = follower.followPathAuto(targetHeading, odometry, drive);
            }else {
                pathing = follower.followPathAuto(targetHeading, odometry, drive, 4);
            }

        }else if (Math.abs(CE4F.getX() - odometry.X) < collectionError && Math.abs(CE4F.getY() - odometry.Y) < collectionError && !pathing){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
            collection.updateIntakeHeight();

            pathing = true;

            onlyOnce = false;

            delivering = true;

            armOver = false;

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

        if (collection.getIntakeCurrentUse() > 4000 && !reversingIntake){
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

        if (gripperControl.milliseconds() > (timeChanger+200) && gripperControl.milliseconds() < (timeChanger+400) && gotTwo){

            collection.setState(Collection.intakePowerState.reversed);
            collection.updateIntakeState();

        }

        if (gripperControl.milliseconds() > (timeChanger+400) && gripperControl.milliseconds() < (timeChanger+600) && gotTwo){

            collection.setState(Collection.intakePowerState.off);
            collection.updateIntakeState();

            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();

        }

        if (delivering){

            if (odometry.X > extendSlidesDelivery.getX()) {

                onlyOnce = true;

                deliverySlides.DeliverySlides(slidesPosWhitePixels, 1);
                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();

            }

            if (deliverySlides.getCurrentposition() > 200 && !armOver){

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

                phase = Blue_Auto_Right.Phase.finished;

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

                collection.setState(Collection.intakePowerState.reversedHalf);
                collection.updateIntakeState();

            }

            if (Math.abs(DE3F.getX() - odometry.X) < deliveryError && Math.abs(DE3F.getY() - odometry.Y) < deliveryError && !pathing) {

                drive.setAllPower(-0.4);

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

                phase = Blue_Auto_Right.Phase.finished;

                drive.setAllPower(0);

                while (!(delivery.getArmState() == Delivery.armState.collect) || deliverySlides.getCurrentposition() > 20){
                    delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardLeft, odometry);
                }

            }

        }

        if (pathing){

            pathing = follower.followPathAuto(targetHeading, odometry, drive);

        }else if (Math.abs(CE4F.getX() - odometry.X) < collectionError && Math.abs(CE4F.getY() - odometry.Y) < collectionError && !pathing){

            drive.setAllPower(0);

            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
            collection.updateIntakeHeight();

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

    Blue_Auto_Right.Phase phase = Phase.preload;

    Blue_Auto_Right.Build build = Build.notBuilt;

    Blue_Auto_Right.Auto auto = Blue_Auto_Right.Auto.preload;

    Preload preload = Preload.purple;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        collect.threePoints(CS1F, CC1F, CE1F);
        collect.twoPoints(CS2F, CE2F);
        collect.threePoints(CS3F, CC3F, CE3F);
        collect.fourPoints(CS4F, CC4F, CCC4F, CE4F, true, 0.5);

        deliver.threePoints(DS1F, DC1F, DE1F);
        deliver.twoPoints(DS2F, DE2F);
        deliver.threePoints(DS3F, DC3F, DE3F, true);

        while(!lockIn){

            telemetry.addData("Auto activated", auto);
            telemetry.addData("press y for 2+1", "");
            telemetry.addData("press a for 2+3", "");
            telemetry.addData("press b for 2+5", "");
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
            yellow.twoPoints(DYS2F, DYE2F);
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
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();
                                }

                                if (pathing){

                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.015, 2);

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

                                    if (Math.abs(extendSlidesDelivery.getX() - odometry.X) < HeadingControlError && deliverySlides.getCurrentposition() < 50){

                                        deliverySlides.DeliverySlides(slidesPosYellowPixel, 1);

                                        delivery.setGripperState(Delivery.GripperState.closed);
                                        delivery.updateGrippers();

                                    }

                                    if (deliverySlides.getCurrentposition() > 150){

                                        delivery.setArmTargetState(Delivery.armState.readyForDelivering);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

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

        } else if (propPos == 2) {

            //Build purple preload path
            purple.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true, 0.8);

            //Build yellow preload path with three segments
            yellow.threePoints(DYS1S, DYC1S, DYE1S);
            yellow.twoPoints(DYS2S, DYE2S);
            yellow.threePoints(DYS3S, DYC3S, DYE3S, true, 1);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            // while loop to keep auto running while tasks are not complete
            while (!(phase == Blue_Auto_Right.Phase.finished)){

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
                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.015, 2);

                                    //turn to leave purple pixel on spike mark
                                    if (Math.abs(leavePurpleHeadingS.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingS.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                        targetHeading = 200;
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

                                        delivery.setArmTargetState(Delivery.armState.readyForDelivering);
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

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

        }else if (propPos == 3) {

            purple.twoPoints(DPS1T, DPE1T, true);
            thirdPosSecond.threePoints(DPS2T, DPC2T, DPE2T, true);

            yellow.threePoints(DYS1T, DYC1T, DYE1T);
            yellow.twoPoints(DYS2T, DYE2T);
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
                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();

                                }

                                if (pathing){

                                    pathing = follower.followPathAutoHeading(targetHeading, odometry, drive, 0.015, 2);

                                    if (Math.abs(leavePurpleHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(leavePurpleHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 270){
                                        targetHeading = 300;
                                    }

                                    if (Math.abs(oneEightyHeadingT.getX() - odometry.X) < HeadingControlError && Math.abs(oneEightyHeadingT.getY() - odometry.Y) < HeadingControlError && targetHeading == 305){
                                        targetHeading = 180;
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
                                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.yellow2Blue, odometry);

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
