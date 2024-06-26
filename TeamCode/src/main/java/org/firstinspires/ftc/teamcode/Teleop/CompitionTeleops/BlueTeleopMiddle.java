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
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.teleopPathBuilder;
import org.firstinspires.ftc.teamcode.Teleop.TeleopPathing;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Sensors;
import org.firstinspires.ftc.teamcode.hardware.planeLauncher;

import java.util.List;
import java.util.Objects;


@TeleOp
public class BlueTeleopMiddle extends OpMode implements TeleopPathing {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(90, 23, 180);

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
    teleopPathBuilder backboard = new teleopPathBuilder();

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

    boolean RightTrigger = false;

    int snapPos;

    ElapsedTime elapsedTime = new ElapsedTime();

    ElapsedTime closeRight = new ElapsedTime();
    ElapsedTime closeLeft = new ElapsedTime();

//    ElapsedTime sweeper = new ElapsedTime();

    List<LynxModule> allHubs;

    int waitTimeSensors;

    boolean resettingSlides = false;

    boolean turnedOff;

    double lastX;
    double lastY;
    int xCounter;
    int pivotIntakePos = -1;

    boolean reversingIntake = false;
    Collection.intakePowerState previousState = Collection.intakePowerState.off;
    ElapsedTime reverseIntake = new ElapsedTime();

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

        if (gamepad1.left_stick_button && odometry.X > 205 && odometry.X < 300 && odometry.Y < 160){

            double xerror = Math.abs(getRealCoords(305) - robotPos.getX());
            double yerror = Math.abs(getRealCoords(85) - robotPos.getY());

            if (xerror < 4){

            }else {

                backboard.buildPathLine(robotPos, new Vector2D(getRealCoords(305), getRealCoords(85)));

                follower.setPath(backboard.followablePath, backboard.pathingVelocity);

                pathing = true;

            }

        }

//        if (gamepad1.left_trigger > 0.5 && odometry.X > 205){
//
//            double xerror = Math.abs(getRealCoords(250) - robotPos.getX());
//
//            if (xerror < 4){
//
//            }else {
//
//                droneLauncher.buildPathLine(robotPos, new Vector2D(getRealCoords(250), robotPos.getY()));
//
//                follower.setPath(droneLauncher.followablePath, droneLauncher.pathingVelocity);
//
//                pathing = true;
//
//            }
//
//        }

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

//        if (odometry.X < 240 && lastX > 240 && headingLock){
//            headingLock = false;
//        } else if (odometry.Y > 150 && lastY < 150 && headingLock) {
//            headingLock = false;
//        }

        inBackboardArea = odometry.X > 210 && odometry.Y < 150 && odometry.X < 340;

        if(firstSnap){
            snapPos = findClosestPosBlue(robotPos);
            firstSnap = false;
        }

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            headingLock = false;
            pathing = false;
        }

        if(gamepad1.left_trigger > 0){
            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeState();
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
            horizontal = -gamepad1.right_stick_x;

            if (headingLock){
                pivot = follower.getTurnPowerTeleop(180, odometry.heading);
            } else if (!snapToBackboard) {
                pivot = gamepad1.left_stick_x;
            }

            double slowPivot;

            if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery || Math.abs(vertical) > 0.5 || Math.abs(horizontal) > 0.5){
                slowPivot = 0.4;
            }else {
                slowPivot = 0.6;
            }

            if (pivot > slowPivot){
                pivot = slowPivot;
            } else if (pivot < -slowPivot) {
                pivot = -slowPivot;
            }

            double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

            drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
            drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
            drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
            drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        }

        /**intake code*/
        //this is to toggle fully up and fully down on the intake
        if (currentGamepad2.start && !previousGamepad2.start && collection.getIntakeHeightRight() > 0){

            collection.setState(Collection.intakePowerState.on);

            collection.setIntakeHeight(Collection.intakeHeightState.collect);

            collection.updateIntakeHeight();

            collection.updateIntakeState();

        } else if (currentGamepad2.start && !previousGamepad2.start && collection.getIntakeHeightRight() < 0.5) {

            collection.setState(Collection.intakePowerState.off);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);

            collection.updateIntakeHeight();

            collection.updateIntakeState();

        }

//        if (currentGamepad1.left_trigger > 0 && !(previousGamepad1.left_trigger > 0)) {
//
////            collection.IntakeHeightRight.setPosition(collection.IntakeHeightRight.getPosition() - 0.005);
//
//        }

        if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)){

//            collection.IntakeHeightRight.setPosition(collection.IntakeHeightRight.getPosition()+0.005);

            pivotIntakePos++;

            if (pivotIntakePos == 5){
                pivotIntakePos = 0;
            }

            if(pivotIntakePos == 0){
                collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
            }
            else if(pivotIntakePos == 1){
                collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
            }
            else if(pivotIntakePos == 2){
                collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
            }
            else if(pivotIntakePos == 3){
                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
            }
            else if(pivotIntakePos == 4){
                collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
            }

        }

        if (currentGamepad2.y) {
            collection.setState(Collection.intakePowerState.reversed);
        } else if (previousGamepad2.y) {
            collection.setState(Collection.intakePowerState.off);
        }

        //toggle intake
        if (currentGamepad2.back && !previousGamepad2.back && collection.getIntakePower() == 0) {
            collection.setState(Collection.intakePowerState.on);
        } else if (currentGamepad2.back && !previousGamepad2.back && Math.abs(collection.getIntakePower()) > 0) {
            collection.setState(Collection.intakePowerState.off);
        }

        /**Slide code*/
        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

        /**Arm code*/
        //Move to delivery position
        if ((gamepad1.dpad_up || gamepad2.dpad_up) && deliverySlides.getCurrentposition() > 200){
            delivery.setArmTargetState(Delivery.armState.delivery);
        } else if ((gamepad1.dpad_down || gamepad2.dpad_down) && deliverySlides.getCurrentposition() > 200) {
            delivery.setArmTargetState(Delivery.armState.collect);
        }

        /**gripper code*/

        if (gamepad1.start) {
            delivery.setGripperState(Delivery.GripperState.closed);
            collection.setState(Collection.intakePowerState.off);
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

                delivery.updateGrippers();
                collection.updateIntakeHeight();

            }

            delivery.setGripperState(Delivery.GripperState.openDeliver);

        }

        if (gamepad2.x) {
            collection.setState(Collection.intakePowerState.off);
            delivery.setGripperState(Delivery.GripperState.closed);
        }

        switch (delivery.getLeftgripperstate()){

            case closed:

                if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.collect) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.open);
                } else if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.openDeliver);
                }

                break;
            case open:

                if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                }

                break;
            case openDeliver:

                if (currentGamepad2.right_bumper && !previousGamepad2.right_bumper) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                }

                break;
            default:

        }

        switch (delivery.getRightgripperstate()){

            case closed:

                if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.collect) {
                    delivery.setRightGripperState(Delivery.rightGripperState.open);
                } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
                    delivery.setRightGripperState(Delivery.rightGripperState.openDeliver);
                }

                break;
            case open:

                if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
                    delivery.setRightGripperState(Delivery.rightGripperState.closed);
                }

                break;
            case openDeliver:

                if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper) {
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

            deliverySlides.DeliverySlides(deliverySlides.getCurrentposition() + 50, 0.8);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

            RightTrigger = true;

        }

        if (RightTrigger && delivery.getArmState() == Delivery.armState.collect){

            RightTrigger = false;

            deliverySlides.runToPosition(0);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

        }

        if (gamepad2.left_trigger > 0 && deliverySlides.getCurrentposition() < 1800){

            delivery.setArmTargetState(Delivery.armState.collect);

            deliverySlides.runToPosition(2050);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

        } else if (gamepad2.left_trigger > 0 && deliverySlides.getCurrentposition() > 1800) {

            delivery.setArmTargetState(Delivery.armState.collect);

            deliverySlides.runToPosition(600);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);
        }

        //else if (gamepad2.left_trigger > 0 && deliverySlides.getCurrentposition() > 1300){
//
//            collection.setIntakeHeight(Collection.intakeHeightState.startingBox);
//
//            deliverySlides.DeliverySlides(800, -1);
//
//            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);
//
//        }

        if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button) {
            planelauncher.setTrigger(0.2);
        }

//        }else if (currentGamepad2.right_stick_button && !previousGamepad2.right_stick_button && planelauncher.getTriggerPosition() > 0.45){
//            planelauncher.setTrigger(0);
//        }

        if (gamepad2.left_stick_y < -0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
            delivery.setRotateClaw(0.55);
        }else if (gamepad2.left_stick_x > 0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
            delivery.setRotateClaw(0.18);
        } else if (gamepad2.left_stick_x < -0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
            delivery.setRotateClaw(0.81);
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

//        if (sensors.armSensor.isPressed() && delivery.getArmState() == Delivery.armState.delivery){
//            delivery.setGripperState(Delivery.GripperState.openDeliver);
//            delivery.setArmTargetState(Delivery.armState.collect);
//        }

        if (gamepad1.right_bumper){
            collection.setState(Collection.intakePowerState.onHalf);
            collection.updateIntakeState();
        }

        if (gamepad1.b){
            sensors.getDetections(telemetry);
            resetOdo();
        }

        odometry.update();
        collection.updateIntakeState();

        //update delivery state
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad2);
        delivery.updateGrippers();

        RobotLog.d("loop time: " + loopTime);

        telemetry.addData("X", odometry.X);
        telemetry.addData("Y", odometry.Y);
        telemetry.addData("heading", odometry.heading);

        telemetry.addData("intake current draw", collection.getIntakeCurrentUse());
        telemetry.addData("loop time", loopTime);
        telemetry.addData("backboard sensor", sensors.armSensor.isPressed());

        telemetry.addData("intake height", collection.getHeightState());
        telemetry.addData("intake height servo", collection.getIntakeHeightRight());

        telemetry.addData("main pivot pos",delivery.getMainPivotPosition());
        telemetry.addData("second rotate", delivery.getSecondRotatePosition());
        telemetry.addData("slides position", deliverySlides.getCurrentposition());
        telemetry.addData("right slides position", deliverySlides.Right_Slide.getCurrentPosition());
        telemetry.addData("left slides position", deliverySlides.Left_Slide.getCurrentPosition());
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
        delivery.initArmMiddle(hardwareMap);
        deliverySlides.init(hardwareMap);

        drive.init(hardwareMap);
        odometry.init(hardwareMap);
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

    }

    public void collectPixels(){

        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
        collection.updateIntakeHeight();

        try {
            Thread.sleep(1000);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        boolean gotTwo;

        delivery.ArmExtension.setPosition(0.96);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
        collection.updateIntakeHeight();

        gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

        if (gotTwo){
            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();
        }else {

            try {
                Thread.sleep(1000);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
            collection.updateIntakeHeight();

            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

            if (gotTwo){
                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();
            }else {
                try {
                    Thread.sleep(1000);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }

                gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

                if (gotTwo){
                    delivery.setGripperState(Delivery.GripperState.closed);
                    delivery.updateGrippers();
                }else {
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException e) {
                        throw new RuntimeException(e);
                    }

                    delivery.setGripperState(Delivery.GripperState.closed);
                    delivery.updateGrippers();
                }
            }

        }


        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        collection.setState(Collection.intakePowerState.reversedHalf);
        collection.updateIntakeState();

        collection.setIntakeHeight(Collection.intakeHeightState.startingBox);
        collection.updateIntakeHeight();

        try {
            Thread.sleep(200);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();
    }

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

                newPosition = new Vector2D(NewX, NewY);

                odometry.reset(newPosition);

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
