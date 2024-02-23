package org.firstinspires.ftc.teamcode.Teleop.CompitionTeleops;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad2;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad2;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.teleopPathBuilder;
import org.firstinspires.ftc.teamcode.Teleop.TeleopPathing;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.teamcode.hardware._.Sensors;
import org.firstinspires.ftc.teamcode.hardware._.planeLauncher;

import java.util.List;
import java.util.Objects;


@TeleOp
public class BlueTeleop extends OpMode implements TeleopPathing {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(90, 23, 180);

    Odometry odometryArc = new Odometry(90, 23, 180);

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    teleopPathBuilder path1 = new teleopPathBuilder();
    teleopPathBuilder path2 = new teleopPathBuilder();
    teleopPathBuilder path3 = new teleopPathBuilder();
    teleopPathBuilder path4 = new teleopPathBuilder();
    teleopPathBuilder path5 = new teleopPathBuilder();
    teleopPathBuilder path6 = new teleopPathBuilder();
    teleopPathBuilder path7 = new teleopPathBuilder();
    teleopPathBuilder droneLauncher = new teleopPathBuilder();

    Vector2D robotPos = new Vector2D();

    mecanumFollower follower = new mecanumFollower();

    planeLauncher planelauncher = new planeLauncher();

    Sensors sensors = new Sensors();

    boolean pathing = false;

    boolean headingLock = false;

    boolean snapToBackboard = false;

    int counter;

    double lastLoopTime;

    double loopTime;

    boolean inBackboardArea = false;

    boolean firstSnap = false;

    int snapPos;

    ElapsedTime elapsedTime = new ElapsedTime();

    ElapsedTime closeRight = new ElapsedTime();
    ElapsedTime closeLeft = new ElapsedTime();

    ElapsedTime sweeper = new ElapsedTime();

    List<LynxModule> allHubs;

    int waitTimeSensors;

    boolean resettingSlides = false;

    boolean turnedOff;

    double lastX;
    double lastY;
    int xCounter;

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        counter++;

        if (counter > 50){
            counter = 0;
        }

        loopTime = elapsedTime.milliseconds() - lastLoopTime;

        lastLoopTime = elapsedTime.milliseconds();

        robotPos.set(odometry.X, odometry.Y);

        //copy to gamepads
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);

        currentGamepad2.copy(gamepad2);

        /**drive code*/

        if (gamepad1.left_trigger > 0.5 && odometry.X > 205){

            double xerror = Math.abs(getRealCoords(250) - robotPos.getX());

            if (xerror < 4){

            }else {

                droneLauncher.buildPathLine(robotPos, new Vector2D(getRealCoords(250), robotPos.getY()));

                follower.setPath(droneLauncher.followablePath, droneLauncher.pathingVelocity);

                pathing = true;

            }

        }

        if (gamepad1.dpad_left){
            headingLock = true;
        } else if (gamepad1.dpad_right) {
            headingLock = false;
        }

        xCounter++;

        if (xCounter > 10){
            xCounter = 0;
            lastX = odometry.X;
        }

        if (odometry.X < 240 && lastX > 240 && headingLock){
            headingLock = false;
        } else if (odometry.Y > 150 && lastY < 150 && headingLock) {
            headingLock = false;
        }

        inBackboardArea = odometry.X > 210 && odometry.Y < 150 && odometry.X < 340;

        if(gamepad1.right_trigger > 0 && inBackboardArea){
            snapToBackboard = true;
        }else {
            snapToBackboard = false;
        }

        if(currentGamepad1.right_trigger > 0.5 && !(previousGamepad1.right_trigger > 0.5) && inBackboardArea){
            firstSnap = true;
        }

        if(firstSnap){
            snapPos = findClosestPosBlue(robotPos);
            firstSnap = false;
        }

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            headingLock = false;
            pathing = false;
        }

        if ((snapToBackboard && gamepad1.right_bumper) || (snapToBackboard && gamepad1.left_bumper) || firstSnap){

            if (currentGamepad1.right_bumper && !(previousGamepad1.right_bumper)) {
                snapPos++;
            }

            if (currentGamepad1.left_bumper && !(previousGamepad1.left_bumper)) {
                snapPos--;
            }

            if (snapPos > 7){
                snapPos = 1;
            }

            if (snapPos < 1){
                snapPos = 7;
            }

        }

        if (snapToBackboard && !pathing){

            if(snapPos == 1){

                double xerror = Math.abs(threeLeftBlue.getX() - robotPos.getX());
                double yerror = Math.abs(threeLeftBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path1.buildPathLine(robotPos, threeLeftBlue);

                    follower.setPath(path1.followablePath, path1.pathingVelocity);

                    pathing = true;
                }

            }else if(snapPos == 2){

                double xerror = Math.abs(twoLeftBlue.getX() - robotPos.getX());
                double yerror = Math.abs(twoLeftBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path2.buildPathLine(robotPos, twoLeftBlue);

                    follower.setPath(path2.followablePath, path2.pathingVelocity);

                    pathing = true;
                }

            }else if(snapPos == 3){

                double xerror = Math.abs(oneLeftBlue.getX() - robotPos.getX());
                double yerror = Math.abs(oneLeftBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path3.buildPathLine(robotPos, oneLeftBlue);

                    follower.setPath(path3.followablePath, path3.pathingVelocity);

                    pathing = true;
                }

            }else if(snapPos == 4){

                double xerror = Math.abs(middleBlue.getX() - robotPos.getX());
                double yerror = Math.abs(middleBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path4.buildPathLine(robotPos, middleBlue);

                    follower.setPath(path4.followablePath, path4.pathingVelocity);

                    pathing = true;
                }

            }else if(snapPos == 5){

                double xerror = Math.abs(oneRightBlue.getX() - robotPos.getX());
                double yerror = Math.abs(oneRightBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path5.buildPathLine(robotPos, oneRightBlue);

                    follower.setPath(path5.followablePath, path5.pathingVelocity);

                    pathing = true;
                }

            }else if(snapPos == 6){

                double xerror = Math.abs(twoRightBlue.getX() - robotPos.getX());
                double yerror = Math.abs(twoRightBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path6.buildPathLine(robotPos, twoRightBlue);

                    follower.setPath(path6.followablePath, path6.pathingVelocity);

                    pathing = true;
                }

            }else if(snapPos == 7){

                double xerror = Math.abs(threeRightBlue.getX() - robotPos.getX());
                double yerror = Math.abs(threeRightBlue.getY() - robotPos.getY());

                if (xerror < 2 && yerror < 2){

                }else {
                    path7.buildPathLine(robotPos, threeRightBlue);

                    follower.setPath(path7.followablePath, path7.pathingVelocity);

                    pathing = true;
                }

            }

        }

        if (pathing && atRest(gamepad1)){

            pathing = follower.followPathTeleop(180, odometry, drive);

        }else {

            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;

            if (headingLock){
                pivot = follower.getTurnPowerTeleop(180, odometry.heading);
            } else if (!snapToBackboard) {
                pivot = gamepad1.left_stick_x;
            }

            double slowPivot = 0.5;

            if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
                if (pivot > slowPivot){
                    pivot = slowPivot;
                } else if (pivot < -slowPivot) {
                    pivot = -slowPivot;
                }
            }

            double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

            drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
            drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
            drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
            drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        }

        /**intake code*/

        //this is to toggle fully up and fully down on the intake
        if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && collection.getIntakeHeight() > 0){
            collection.setState(Collection.intakePowerState.on);
            collection.setIntakeHeight(Collection.intakeHeightState.collect);
            collection.updateIntakeHeight();
            collection.updateIntakeState();
        } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && collection.getIntakeHeight() < 0.5) {
            collection.setState(Collection.intakePowerState.off);
            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();
            collection.updateIntakeState();
        }

//        if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)){
//
//            pivotIntakePos++;
//
//            if (pivotIntakePos == 5){
//                pivotIntakePos = 0;
//            }
//
//            if(pivotIntakePos == 0){
//                collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
//            }
//            else if(pivotIntakePos == 1){
//                collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
//            }
//            else if(pivotIntakePos == 2){
//                collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
//            }
//            else if(pivotIntakePos == 3){
//                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
//            }
//            else if(pivotIntakePos == 4){
//                collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
//            }
//
//            collection.updateIntakeHeight();
//
//        }

        //reverse intake
        if (currentGamepad2.y) {
            collection.setState(Collection.intakePowerState.reversed);
        } else if (previousGamepad2.y) {
            collection.setState(Collection.intakePowerState.off);
        }

        //toggle intake
        if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && collection.getIntakePower() == 0) {
            collection.setState(Collection.intakePowerState.on);
        } else if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && Math.abs(collection.getIntakePower()) > 0) {
            collection.setState(Collection.intakePowerState.off);
        }

        /**Slide code*/

        Delivery.GripperState slidesGripppers = deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        if (slidesGripppers == null){
        }else {
            delivery.setGripperState(slidesGripppers);
        }

        /**Pivot code*/

        //Move to delivery position
        if (gamepad2.dpad_up || gamepad1.dpad_up && deliverySlides.getCurrentposition() > 150){
            delivery.setArmTargetState(Delivery.armState.delivery);
        } else if (gamepad2.dpad_down || gamepad1.dpad_down && deliverySlides.getCurrentposition() > 100) {
            delivery.setArmTargetState(Delivery.armState.collect);
        }

        /**gripper code*/

        if (gamepad1.start) {
            delivery.setGripperState(Delivery.GripperState.closed);
        }

        if (gamepad1.back) {
            if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
                delivery.setGripperState(Delivery.GripperState.openDeliver);
            }
            delivery.setGripperState(Delivery.GripperState.open);
        }

        if (gamepad2.b) {
            if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.collect){
                collection.setState(Collection.intakePowerState.on);
                collection.setIntakeHeight(Collection.intakeHeightState.collect);
                delivery.setGripperState(Delivery.GripperState.open);
            }
            delivery.setGripperState(Delivery.GripperState.openDeliver);
        }

        if (gamepad2.x) {
            collection.setState(Collection.intakePowerState.off);
            delivery.setGripperState(Delivery.GripperState.closed);
        }


        switch (delivery.getLeftgripperstate()){

            case closed:
                if (currentGamepad2.start && !previousGamepad2.start && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.collect) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.open);
                } else if (currentGamepad2.start && !previousGamepad2.start && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.openDeliver);
                }
                break;
            case open:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setRightGripperState(Delivery.rightGripperState.closed);
                }
                break;
            case openDeliver:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setRightGripperState(Delivery.rightGripperState.closed);
                }
                break;
            default:

        }

        switch (delivery.getRightgripperstate()){

            case closed:
                if (currentGamepad2.back && !previousGamepad2.back && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.collect) {
                    delivery.setRightGripperState(Delivery.rightGripperState.open);
                } else if (currentGamepad2.back && !previousGamepad2.back && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
                    delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
                }
                break;
            case open:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setRightGripperState(Delivery.rightGripperState.closed);
                }
                break;
            case openDeliver:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setRightGripperState(Delivery.rightGripperState.closed);
                }
                break;
            default:

        }

        if(collection.getIntakePower() > 0 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.collect){

            if (sensors.RightClawSensor.isPressed()){
                closeRight.reset();
            }

            if (sensors.LeftClawSensor.isPressed()){
                closeLeft.reset();
            }

            waitTimeSensors = 600;

           if (closeLeft.milliseconds() > waitTimeSensors && closeRight.milliseconds() > waitTimeSensors) {

               delivery.setGripperState(Delivery.GripperState.closed);
               collection.setState(Collection.intakePowerState.off);
               collection.setIntakeHeight(Collection.intakeHeightState.stowed);
               collection.updateIntakeHeight();
               collection.updateIntakeState();

           }else {

               if(closeLeft.milliseconds() > waitTimeSensors){
                   delivery.setLeftGripperState(Delivery.leftGripperState.closed);
               } else if(closeRight.milliseconds() > waitTimeSensors){
                   delivery.setRightGripperState(Delivery.rightGripperState.closed);
               }

           }
        }

        if (gamepad2.right_trigger > 0){
            delivery.setArmTargetState(Delivery.armState.collect);

            deliverySlides.DeliverySlides(0, -0.5);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);
        }

        if (gamepad2.left_trigger > 0){

            delivery.setArmTargetState(Delivery.armState.collect);

            deliverySlides.DeliverySlides(1450, 0.7);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

        } else if (gamepad2.left_trigger > 0 && deliverySlides.getCurrentposition() > 1300){

            collection.setIntakeHeight(Collection.intakeHeightState.hangStowed);

            deliverySlides.DeliverySlides(800, -0.8);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

        }

        if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button && planelauncher.getTriggerPosition() < 0.1){
            planelauncher.setTrigger(0.6);
        }else if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button && planelauncher.getTriggerPosition() > 0.45){
            planelauncher.setTrigger(0);
        }

        if (gamepad2.left_stick_y < -0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
            delivery.setRotateClaw(0.5);
        }else if (gamepad2.left_stick_x > 0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
            delivery.setRotateClaw(0.3);
        } else if (gamepad2.left_stick_x < -0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
            delivery.setRotateClaw(0.7);
        }

        if (gamepad2.left_stick_button){
            deliverySlides.SlidesBothPower(-0.4);
            resettingSlides = true;
        }

        if (resettingSlides){

            if (deliverySlides.getCurrentDraw() >= 2000){

                deliverySlides.SlidesBothPower(0);
                resettingSlides = false;
                deliverySlides.resetZero();

            }else {

            }

        }

        if (gamepad1.y){
            collection.setSweeperState(Collection.sweeperState.push);
            collection.updateSweeper();
            sweeper.reset();
        } else if (sweeper.milliseconds() > 100 && sweeper.milliseconds() < 200) {
            collection.setSweeperState(Collection.sweeperState.retract);
            collection.updateSweeper();
        }

        if (gamepad1.b){
            sensors.getDetections();
            resetOdo();
        }

        odometry.update();

        //update collection state
        collection.updateIntakeHeight();
        collection.updateIntakeState();

        //update delivery state
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);
        delivery.updateGrippers();

        RobotLog.d("loop time: " + loopTime);

        telemetry.addData("X", odometry.X);
        telemetry.addData("Y", odometry.Y);
        telemetry.addData("heading", odometry.heading);
        telemetry.addData("arm pposition", Range.clip(1.2, 0, 1));
        telemetry.addData("intake current draw", collection.getIntakeCurrentUse());
        telemetry.addData("slides current draw", deliverySlides.getCurrentDraw());
        telemetry.addData("loop time", loopTime);
        telemetry.update();

    }

    @Override
    public void init() {

        allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        planelauncher.init(hardwareMap);

        elapsedTime.reset();

        collection.init(hardwareMap);
        delivery.init(hardwareMap);
        deliverySlides.init(hardwareMap);

        drive.init(hardwareMap);
        odometry.init(hardwareMap);
        odometryArc.init(hardwareMap);
        sensors.init(hardwareMap);

        sensors.initAprilTag(telemetry, false);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();

        collection.setIntakeHeight(Collection.intakeHeightState.collect);

        sensors.portal.setProcessorEnabled(sensors.propDetectionByAmount, false);
    }

    public void init_loop() {

        sensors.getDetections();

        resetOdo();

        odometry.update();

    };

    public void resetOdo(){

        if (!(sensors.rightTag == null)){

            if (sensors.rightTag.id == 1 || sensors.rightTag.id == 2 || sensors.rightTag.id == 3){

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 1){
                    aprilTagOffset = getRealCoords(75);
                }else if (sensors.rightTag.id == 2){
                    aprilTagOffset = getRealCoords(90);
                }else{
                    aprilTagOffset = getRealCoords(105);
                }

                double realNewX = (sensors.rightTag.ftcPose.y * 0.1);
                double realNewY = (sensors.rightTag.ftcPose.x * 0.1);

                NewY = (realNewY + aprilTagOffset)-12;
                NewX = 360 - (realNewX + 45);

                double heading = odometry.getIMUHeading();

                newPosition = new Vector2D(NewX, NewY);

                odometry.reset(newPosition, heading);

                telemetry.addData("X reset pos", NewX);
                telemetry.addData("Y reset pos", NewY);
                telemetry.update();
            }
        }

    }

    public boolean atRest(Gamepad gamepad){
        return gamepad.right_stick_x == 0 && gamepad.right_stick_y == 0 && gamepad.left_stick_x == 0 && gamepad.left_stick_y == 0;
    }
}
