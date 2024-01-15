package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
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

    FtcDashboard dashboard = FtcDashboard.getInstance();

    Telemetry dashboardTelemetry = dashboard.getTelemetry();

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry(210, 337, 270);

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

    double autoDropLastTime;
    boolean autoDrop = false;

    String backboardPosition;
    boolean Confirmed = false;

    int counter;

    double lastLoopTime;

    double loopTime;

    boolean backboardSafe = false;

    ElapsedTime elapsedTime = new ElapsedTime();

    List<LynxModule> allHubs;

    double intakePos = 0;

    @Override
    public void loop() {

        for (LynxModule hub : allHubs) {
            hub.clearBulkCache();
        }

        counter++;

        if (counter > 50){
            counter = 0;
            loopTime = elapsedTime.milliseconds() - lastLoopTime;
        }

        lastLoopTime = elapsedTime.milliseconds();

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
            pathing = follower.followPathTeleop(true, targetHeading, true, odometry, drive, telemetry);
        }else {

            vertical = -gamepad1.right_stick_y;
            horizontal = gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;

            if (pivot > 0.6){
                pivot = 0.6;
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
            collection.setIntakeHeight(Collection.intakeHeightState.collect);
            collection.updateIntakeHeight();
        } else if (currentGamepad2.left_bumper && !previousGamepad2.left_bumper && collection.getIntakeHeight() < 0.5) {
            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();
        }

        if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)){

            intakePos += 0.01;

            collection.IntakeHeight.setPosition(intakePos);

            if(intakePos > 0.3){
                intakePos = 0;
            }

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

        Delivery.targetGripperState slidesGripppers = deliverySlides.updateSlides(gamepad1, gamepad2);

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
            delivery.setGripperState(Delivery.targetGripperState.closeBoth);
        }

        if (gamepad1.back) {
            delivery.setGripperState(Delivery.targetGripperState.openBoth);
        }

        if (gamepad2.b) {
            delivery.setGripperState(Delivery.targetGripperState.openBoth);
        }

        if (gamepad2.x) {
            delivery.setGripperState(Delivery.targetGripperState.closeBoth);
        }


        switch (delivery.getLeftgripperstate()){
            case closed:
                if (currentGamepad2.start && !previousGamepad2.start) {
                    delivery.setGripperState(Delivery.targetGripperState.openRight);
                }
                break;
            case open:
                if (currentGamepad2.start && !previousGamepad2.start) {
                    delivery.setGripperState(Delivery.targetGripperState.closeRight);
                }
                break;
            default:
        }

        switch (delivery.getRightgripperstate()){
            case closed:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setGripperState(Delivery.targetGripperState.openLeft);
                }
                break;
            case open:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setGripperState(Delivery.targetGripperState.closeLeft);
                }
                break;
            default:
        }

       if(collection.getIntakePower() > 0){

           double rightClawReading = sensors.RightClawSensor.getDistance(DistanceUnit.MM);
           double leftClawReading = sensors.LeftClawSensor.getDistance(DistanceUnit.MM);

           if (rightClawReading < 75 && leftClawReading < 75) {
               delivery.setGripperState(Delivery.targetGripperState.closeBoth);
           }else {
               if(leftClawReading < 75){
                   delivery.setGripperState(Delivery.targetGripperState.closeLeft);
               } else if(rightClawReading < 75){
                   delivery.setGripperState(Delivery.targetGripperState.closeRight);
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

            deliverySlides.DeliverySlides(1000, 0.7);

            deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);
        }

        if (gamepad1.left_trigger > 0){
            planelauncher.setTrigger(0.5);
        }

        odometry.update();

        //update collection state
//        collection.updateIntakeHeight();
        collection.updateIntakeState();

        //update delivery state
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry, gamepad2);
        delivery.updateGrippers();

        telemetry.addData("intake current draw", collection.getIntakeCurrentUse());
        telemetry.addData("Intakeheight", collection.getIntakeHeight());
        telemetry.addData("Slide state ", deliverySlides.getSlideState());
        telemetry.addData("main pivot position", delivery.getMainPivotPosition());
        telemetry.addData("loop time", loopTime);
        telemetry.update();

    }

    @Override
    public void init() {

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

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

        sensors.initAprilTag();

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();

        collection.setIntakeHeight(Collection.intakeHeightState.collect);
    }

    public void init_loop() {

        sensors.getDetections();

        resetOdo();

        odometry.update();
        telemetry.addData("x", odometry.X);
        telemetry.addData("y", odometry.Y);
        telemetry.update();
    };

    public void resetOdo(){

        if (!(sensors.rightTag == null)){

            if (sensors.rightTag.id == 4 || sensors.rightTag.id == 5 || sensors.rightTag.id == 6){

                double NewY;
                double NewX;

                double aprilTagOffset;

                Vector2D newPosition;

                if (sensors.rightTag.id == 4){
                    aprilTagOffset = getRealCoords(75);
                }else if (sensors.rightTag.id == 5){
                    aprilTagOffset = getRealCoords(90);
                }else{
                    aprilTagOffset = getRealCoords(105);
                }

                double realNewY = (Math.cos(sensors.rightTag.ftcPose.bearing)) * (sensors.rightTag.ftcPose.x * 0.1);
                double realNewX = (Math.cos(sensors.rightTag.ftcPose.bearing)) * (sensors.rightTag.ftcPose.y * 0.1);

                NewY = aprilTagOffset + realNewY;
                NewX = 360 - realNewX;

                newPosition = new Vector2D((NewX -5) - 46, (NewY - 11));

                odometry.reset(newPosition);

                telemetry.addData("rightTag.ftcPose.yaw", sensors.rightTag.ftcPose.yaw);
                telemetry.addData("rightTag.ftcPose.y", sensors.rightTag.ftcPose.y);
                telemetry.addData("rightTag.ftcPose.x", sensors.rightTag.ftcPose.x);
                telemetry.addData("X reset pos", newPosition.getX());
                telemetry.addData("Y reset pos", newPosition.getY());

            }
        }

    }
}
