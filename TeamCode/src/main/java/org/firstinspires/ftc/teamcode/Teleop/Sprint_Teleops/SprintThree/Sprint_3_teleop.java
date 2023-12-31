package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad2;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad2;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.teleopPathBuilder;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;


@TeleOp
public class Sprint_3_teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Odometry odometry = new Odometry();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    teleopPathBuilder pathBuilder = new teleopPathBuilder();

    Vector2D robotPos = new Vector2D();

    mecanumFollower follower = new mecanumFollower();

    double pivotIntakePos = 0;

    double targetHeading;

    boolean pathing = false;

    @Override
    public void loop() {

        robotPos.set(odometry.X, odometry.Y);

        //copy to gamepads
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        previousGamepad2.copy(currentGamepad2);

        currentGamepad2.copy(gamepad2);

        /**drive code*/

        //drive to backboard
        if (gamepad1.b){

            Vector2D targetPoint = new Vector2D(getRealCoords(300), getRealCoords(270));

            pathBuilder.buildPath(teleopPathBuilder.TeleopPath.red, robotPos, targetPoint);

            targetHeading = 180;

            pathing = true;

            follower.setPath(pathBuilder.followablePath, pathBuilder.pathingVelocity);
        }

//        //drive to collection
//        if (gamepad1.a){
//
//            Vector2D targetPoint = new Vector2D(getRealCoords(38), getRealCoords(88));
//
//            pathBuilder.buildPath(teleopPathBuilder.TeleopPath.red, robotPos, targetPoint);
//
//            targetHeading = 270;
//
//            pathing = true;
//
//            follower.setPath(pathBuilder.followablePath, pathBuilder.pathingVelocity);
//        }

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            pathing = false;
        }

        if (pathing && gamepad1.atRest()){
            follower.followPathTeleop(true, targetHeading, false, odometry, drive);
        }else {

            vertical = -gamepad1.right_stick_y;
            horizontal = -gamepad1.right_stick_x;
            pivot = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

            drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
            drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
            drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
            drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        }


        /**intake code*/

        //this is to toggle fully up and fully down on the intake
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && collection.getIntakeHeight() > 0){
            collection.setIntakeHeight(Collection.intakeHeightState.collect);
        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && collection.getIntakeHeight() < 0.5) {
            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
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
        if (currentGamepad1.y && !previousGamepad1.y && collection.getIntakePower() > 0.1) {
            collection.setState(Collection.intakePowerState.reversed);
        }

        //toggle intake
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && collection.getIntakePower() == 0) {
            collection.setState(Collection.intakePowerState.on);
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && Math.abs(collection.getIntakePower()) > 0) {
            collection.setState(Collection.intakePowerState.off);
        }

        /**Slide code*/
        deliverySlides.updateSlides(gamepad1, gamepad2);

        /**Pivot code*/

        //Move to delivery position
        if (gamepad1.dpad_up && deliverySlides.getCurrentposition() > 150){
            delivery.setArmTargetState(Delivery.armState.delivery);
        } else if (gamepad1.dpad_down && deliverySlides.getCurrentposition() > 100) {
            delivery.setArmTargetState(Delivery.armState.collect);
        }

        /**gripper code*/

        if (gamepad1.start) {
            delivery.setGripperState(Delivery.targetGripperState.closeBoth);
        }

        if (gamepad1.back) {
            delivery.setGripperState(Delivery.targetGripperState.openBoth);
        }

        switch (delivery.getLeftgripperstate()){
            case closed:
                if (currentGamepad2.start && !previousGamepad2.start) {
                    delivery.setGripperState(Delivery.targetGripperState.openLeft);
                }
                break;
            case open:
                if (currentGamepad2.start && !previousGamepad2.start) {
                    delivery.setGripperState(Delivery.targetGripperState.closeLeft);
                }
                break;
            default:
        }

        switch (delivery.getRightgripperstate()){
            case closed:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setGripperState(Delivery.targetGripperState.openRight);
                }
                break;
            case open:
                if (currentGamepad2.back && !previousGamepad2.back) {
                    delivery.setGripperState(Delivery.targetGripperState.closeRight);
                }
                break;
            default:
        }

        //update collection state
        collection.updateIntakeHeight();
        collection.updateIntakeState();

        //update delivery state
        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, gamepad1, telemetry);
        delivery.updateGrippers();

        telemetry.addData("arm state", delivery.getArmState());
        telemetry.addData("slide position", deliverySlides.getCurrentposition());
        telemetry.addData("main pivot position", delivery.getMainPivotPosition());
        telemetry.update();


    }

    @Override
    public void init() {
        collection.init(hardwareMap);
        delivery.init(hardwareMap);
        deliverySlides.init(hardwareMap);

        drive.init(hardwareMap);
        odometry.init(hardwareMap);

        previousGamepad1 = new Gamepad();
        currentGamepad1 = new Gamepad();

        previousGamepad2 = new Gamepad();
        currentGamepad2 = new Gamepad();

        collection.setIntakeHeight(Collection.intakeHeightState.collect);
    }
}
