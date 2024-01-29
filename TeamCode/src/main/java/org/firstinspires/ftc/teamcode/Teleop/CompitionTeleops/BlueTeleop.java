package org.firstinspires.ftc.teamcode.Teleop.CompitionTeleops;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad2;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad2;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.teleopPathBuilder;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Sensors;
import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.planeLauncher;

import java.util.List;
import java.util.Objects;


@TeleOp
public class BlueTeleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(210, 337, 180);

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    teleopPathBuilder pathBuilder = new teleopPathBuilder();

    Vector2D robotPos = new Vector2D();

    mecanumFollower follower = new mecanumFollower();

    planeLauncher planelauncher = new planeLauncher();

    Sensors sensors = new Sensors();

    double pivotIntakePos = 0;

    double targetHeading;

    boolean pathing = false;

    String backboardPosition;
    boolean Confirmed = false;

    int counter;

    double lastLoopTime;

    double loopTime;

    boolean backboardSafe = false;

    ElapsedTime elapsedTime = new ElapsedTime();

    ElapsedTime closeRight = new ElapsedTime();
    ElapsedTime closeLeft = new ElapsedTime();
    ElapsedTime resetOdo = new ElapsedTime();

    List<LynxModule> allHubs;

    int waitTimeSensors;

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        counter++;

        if (counter > 50){
            counter = 0;
            lastLoopTime = loopTime;
            loopTime = elapsedTime.milliseconds() - lastLoopTime;
        }

        robotPos.set(odometry.X, odometry.Y);

        //copy to gamepads
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);

        currentGamepad2.copy(gamepad2);

        /**drive code*/

//        //drive to backboard
//        if (gamepad1.b){
//
//            Vector2D targetPoint = new Vector2D(getRealCoords(220), getRealCoords(270));
//
//            pathBuilder.buildPath(teleopPathBuilder.TeleopPath.red, robotPos, targetPoint);
//
//            targetHeading = 180;
//
//            pathing = true;
//
//            follower.setPath(pathBuilder.followablePath, pathBuilder.pathingVelocity);
//
//        }

//        //drive to collection
//        if (gamepad2.a){
//
//            Vector2D targetPoint = new Vector2D(getRealCoords(55), getRealCoords(88));
//
//            pathBuilder.buildPath(teleopPathBuilder.TeleopPath.red, robotPos, targetPoint);
//
//            targetHeading = 270;
//
//            pathing = true;
//
//            follower.setPath(pathBuilder.followablePath, pathBuilder.pathingVelocity);
//        }

        //set target position for backboard

        if (gamepad2.dpad_up ){
            backboardPosition = "Center";
        }else if (gamepad2.x){
            backboardPosition = "Left";
        }else if (gamepad2.b){
            backboardPosition = "Right";
        }

        //confirm target
        if (gamepad2.left_trigger > 0){
            Confirmed = true;
        }

        backboardSafe = odometry.X > 210 && odometry.Y < 180 && odometry.X < 300;

        if (Confirmed && !pathing && backboardSafe){
            if (Objects.equals(backboardPosition, "Left")){
                odometry.Odo_Drive_Teleop(295, 64, 180);
            }else if (Objects.equals(backboardPosition, "Center")){
                odometry.Odo_Drive_Teleop(295, 79, 180);
            }else if (Objects.equals(backboardPosition, "Right")){
                odometry.Odo_Drive_Teleop(295, 94, 180);
            }
        }

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            pathing = false;
            Confirmed = false;
        }

        if (pathing && gamepad1.atRest()){
            pathing = follower.followPathTeleop(targetHeading, odometry, drive);
        }else {

            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;

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

        if (Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery && Math.abs(odometry.getHorizontalVelocity()) > 5){

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

        if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)){

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

            collection.updateIntakeHeight();

        }

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
                if (currentGamepad2.start && !previousGamepad2.start) {
                    delivery.setLeftGripperState(Delivery.leftGripperState.closed);
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

        if (currentGamepad2.a && !previousGamepad2.a && planelauncher.getTriggerPosition() < 0.1){
            planelauncher.setTrigger(0.6);
        }else if (currentGamepad2.a && !previousGamepad2.a && planelauncher.getTriggerPosition() > 0.45){
            planelauncher.setTrigger(0);
        }

        if (gamepad2.left_stick_y < -0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
            delivery.setRotateClaw(0.5);
        }else if (gamepad2.left_stick_x > 0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery){
            delivery.setRotateClaw(0);
        } else if (gamepad2.left_stick_x < -0.9 && Objects.requireNonNull(delivery.getArmState()) == Delivery.armState.delivery) {
            delivery.setRotateClaw(1);
        }

        sensors.getDetections();

        if (gamepad1.b){
            resetOdo.reset();
            double heading = odometry.getIMUHeading();
            odometry.reset(heading);
        }

        if (resetOdo.milliseconds() < 500){
            resetOdo();
        }

        odometry.update();

        //update collection state
        collection.updateIntakeHeight();
        collection.updateIntakeState();

        //update delivery state
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);
        delivery.updateGrippers();


        telemetry.addData("X", odometry.X);
        telemetry.addData("Y", odometry.Y);
        telemetry.addData("heading", odometry.heading);
//        telemetry.addData("distance sensor", sensors.backBoard.getDistance(DistanceUnit.CM));
//        if (sensors.rightTag != null){
//            telemetry.addData("distance camera", ((sensors.rightTag.ftcPose.y)*0.1));
//        }
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

                double heading = odometry.getIMUHeading();

                double realNewX = (sensors.rightTag.ftcPose.y * 0.1);
                double realNewY = (sensors.rightTag.ftcPose.x * 0.1);

                NewY = (realNewY + aprilTagOffset)-12;
                NewX = 360 - (realNewX + 45);

                newPosition = new Vector2D(NewX, NewY);

                odometry.reset(newPosition, heading);

                telemetry.addData("heading", heading);
                telemetry.addData("sensors.rightTag.ftcPose.y ", sensors.rightTag.ftcPose.x);
                telemetry.addData("X reset pos", NewX);
                telemetry.addData("Y reset pos", NewY);
                telemetry.update();

            }
        }

    }
}