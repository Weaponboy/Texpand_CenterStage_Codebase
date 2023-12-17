package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;

@TeleOp
public class Sprint_3_teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    //Leave at 0 start for now
    Odometry odometry = new Odometry();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    DistanceSensor RightClaw;
    DistanceSensor LeftClaw;

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    //in ms
    double timePerDegreeTopPivot = 2.8;

    double smallServoTimePerDegree = 1.6;

    //!!!!!
    //tune these values
    //!!!!

    double collectTopPivotPos = 0.1;
    double deliveryTopPivot = 1;
    double safeTopPivot = 0.3;

    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.97;
    double deliverySecondPivot = 0.3;

    double clawOpen = 0.25;
    double clawClosed = 0;

    double rotateCollect = 0.5;
    double rotateRight = 1;

    double timeToWait;

    boolean closeToCollection;

    ElapsedTime pivotMoveTime = new ElapsedTime();

    public enum SlideState{
        manual,
        moving,
        targetReached
    }

    public enum PivotState{
        deposit,
        collect,
        transitioning
    }

    double pivotIntakePos;

    SlideState slideState = SlideState.manual;
    PivotState pivotState = PivotState.collect;

    boolean intakePos = true;

    @Override
    public void loop() {

        //copy to gamepads
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        /**drive code*/

//        horizontal = -gamepad1.right_stick_x;
//        vertical = -gamepad1.right_stick_y;
//        pivot = gamepad1.left_stick_x;
//
//        double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);
//
//        drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
//        drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
//        drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
//        drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        //just the very basic stuff for now
        double vertical = -gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x * 1.5;
        double pivot = gamepad1.left_stick_x;

        drive.RF.setPower((-pivot + (vertical - horizontal)));
        drive.RB.setPower((-pivot + (vertical + horizontal)));
        drive.LF.setPower((pivot + (vertical + horizontal)));
        drive.LB.setPower((pivot + (vertical - horizontal)));

//        if (odometry.getHorizontalVelocity() > 20 || odometry.getVerticalVelocity() > 20) {
//            delivery.setSecondPivot(safeTopPivot);
//        }

        /**intake code*/

        //this is to toggle fully up and fully down on the intake
        if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && collection.IntakeHeight.getPosition() > 0){
            pivotIntakePos = 0.15;
            collection.IntakeHeight.setPosition(pivotIntakePos);
        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && collection.IntakeHeight.getPosition() < 0.5) {
            pivotIntakePos = 0.7;
            collection.IntakeHeight.setPosition(pivotIntakePos);
        }

        if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)){

            if (collection.IntakeHeight.getPosition() >= 0.7){

            }else {
                pivotIntakePos += 0.05;
                collection.IntakeHeight.setPosition(pivotIntakePos);
            }
        }

        //reverse intake
        if (currentGamepad1.y && !previousGamepad1.y && collection.Intake.getPower() > 0.1) {
            collection.Intake.setPower(-1);
        }

        //toggle intake
        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && collection.Intake.getPower() == 0) {
            collection.Intake.setPower(1);
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && Math.abs(collection.Intake.getPower()) > 0) {
            collection.Intake.setPower(0);
        }



        /**Slide code*/

        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() > 2200;
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() < 5;

        switch (slideState){
            case manual:
                if (gamepad1.x && !SlideSafetyHeight) {
                    SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() > 2200;
                    deliverySlides.SlidesBothPower(0.3);
                } else if (gamepad1.a && !SlideSafetyBottom) {
                    SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() < 5;
                    deliverySlides.SlidesBothPower(-0.3);
                }else {
                    deliverySlides.SlidesBothPower(0.0005);
                }
                break;
            case moving:
                if (deliverySlides.Left_Slide.getCurrentPosition() > (deliverySlides.Left_Slide.getTargetPosition()-10) && deliverySlides.Left_Slide.getCurrentPosition() < (deliverySlides.Left_Slide.getTargetPosition()+10) ){
                    slideState = SlideState.targetReached;
                }
                break;
            case targetReached:
                slideState = SlideState.manual;
                break;
            default:
        }

        /**Pivot code*/

        switch (pivotState){
            case collect:

                //Move to delivery position
                if (gamepad1.dpad_up && deliverySlides.getCurrentposition() > 150){

                    pivotState = PivotState.transitioning;

                    pivotMoveTime.reset();

                    timeToWait = Math.max((Math.abs(delivery.getSecondPivotPosition()-deliverySecondPivot)*180)*timePerDegreeTopPivot, (Math.abs(delivery.getTopPivotPosition()-deliveryTopPivot)*180)*timePerDegreeTopPivot);

                    delivery.setClaws(clawClosed);

                    delivery.setSecondPivot(deliverySecondPivot);

                    delivery.setMainPivot(deliveryTopPivot);

                    delivery.RotateClaw.setPosition(rotateCollect);

                }
//                else if (gamepad1.dpad_up && deliverySlides.getCurrentposition() < 150) {
//
//                    closeToCollection = true;
//
//                    pivotState = PivotState.transitioning;
//
//                    delivery.setMainPivot(safeTopPivot);
//
//                    pivotMoveTime.reset();
//
//                    timeToWait = 100;
//
//                }

                break;
            case deposit:

                //Move to collect position
                if (gamepad1.dpad_down && deliverySlides.getCurrentposition() > 150) {

                    pivotState = PivotState.transitioning;

                    pivotMoveTime.reset();

                    timeToWait = Math.max((Math.abs(delivery.getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegreeTopPivot, (Math.abs(delivery.getTopPivotPosition() - collectTopPivotPos) * 180) * timePerDegreeTopPivot);

                    delivery.setClaws(clawClosed);

                    delivery.setSecondPivot(collectSecondPivot);

                    delivery.setMainPivot(collectTopPivotPos);

                    delivery.RotateClaw.setPosition(rotateCollect);

                }

                break;
            case transitioning:

                if (pivotMoveTime.milliseconds() >= timeToWait && delivery.getTopPivotPosition() < 0.5){
                    pivotState = PivotState.deposit;
                    intakePos = false;
                    delivery.setMainPivot(deliveryTopPivot);
                }

                if (pivotMoveTime.milliseconds() >= timeToWait && delivery.getTopPivotPosition() > 0.5){
                    pivotState = PivotState.collect;
                    intakePos = true;
                    delivery.setMainPivot(collectTopPivotPos);
                }
//
//                if (pivotMoveTime.milliseconds() >= timeToWait && delivery.getTopPivotPosition() < 0.5 && !closeToCollection){
//                    pivotState = PivotState.collect;
//
//                    delivery.setMainPivot(collectTopPivotPos);
//
////                    delivery.LeftClaw.setPosition(1);
////                    delivery.RightClaw.setPosition(1);
//                }

//                if (closeToCollection && pivotMoveTime.milliseconds() >= timeToWait){
//
//                    closeToCollection = false;
//
//                    pivotMoveTime.reset();
//
//                    timeToWait = Math.max((Math.abs(delivery.getSecondPivotPosition()-deliverySecondPivot)*180)*timePerDegreeTopPivot, (Math.abs(delivery.getTopPivotPosition()-deliveryTopPivot)*180)*timePerDegreeTopPivot);
//
//                    delivery.setClaws(clawClosed);
//
//                    delivery.setSecondPivot(deliverySecondPivot);
//
//                    delivery.RotateClaw.setPosition(rotateCollect);
//
//                }

                break;
            default:
        }

        /**gripper code*/

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && delivery.LeftClaw.getPosition() == clawClosed) {
            delivery.LeftClaw.setPosition(clawOpen);
        } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && delivery.LeftClaw.getPosition() == clawOpen) {
            delivery.LeftClaw.setPosition(clawClosed);
        }

        if (gamepad1.start) {
            delivery.RightClaw.setPosition(clawClosed);
            delivery.LeftClaw.setPosition(clawClosed);
        }

        if (gamepad1.back) {
            delivery.RightClaw.setPosition(clawOpen);
            delivery.LeftClaw.setPosition(clawOpen);
        }

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && delivery.RightClaw.getPosition() == clawClosed) {
            delivery.RightClaw.setPosition(clawOpen);
        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && delivery.RightClaw.getPosition() == clawOpen) {
            delivery.RightClaw.setPosition(clawClosed);
        }

//        if (RightClaw.getDistance(DistanceUnit.MM) < 60 && intakePos){
//            delivery.RightClaw.setPosition(clawClosed);
//        }
//
//        if (LeftClaw.getDistance(DistanceUnit.MM) < 60 && intakePos){
//            delivery.LeftClaw.setPosition(clawClosed);
//        }

        telemetry.addData("power intake", collection.Intake.getPower());
//        telemetry.addData("right claw", RightClaw.getDistance(DistanceUnit.MM));
//        telemetry.addData("left claw", LeftClaw.getDistance(DistanceUnit.MM));
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

        pivotMoveTime.reset();

        delivery.setMainPivot(0.1);

        delivery.setSecondPivot(1);

        delivery.RotateClaw.setPosition(0.5);

        delivery.RightClaw.setPosition(clawClosed);
        delivery.LeftClaw.setPosition(clawClosed);

        collection.IntakeHeight.setPosition(0.15);

        RightClaw = hardwareMap.get(DistanceSensor.class, "rightclaw");
        LeftClaw = hardwareMap.get(DistanceSensor.class, "leftclaw");

    }

}
