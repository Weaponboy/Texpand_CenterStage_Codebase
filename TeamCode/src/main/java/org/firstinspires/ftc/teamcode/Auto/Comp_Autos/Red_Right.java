package org.firstinspires.ftc.teamcode.Auto.Comp_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Right", group = "State")
public class Red_Right extends LinearOpMode implements CycleMethods {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.color.red);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redRightBuilder firstPath = new redRightBuilder();

    redRightBuilder collect = new redRightBuilder();

    redRightBuilder deliver = new redRightBuilder();

    mecanumFollower follower = new mecanumFollower();

    boolean builtPath = false;

    enum Phase{
        purple,
        yellow,
        first2,
        second2,
        finished
    }

    enum Auto{
        preload,
        two,
        four
    }

    boolean pathing = false;

    double targetHeading;

    Phase phase = Phase.purple;

    Auto auto = Auto.preload;

    boolean lockIn = false;

    boolean onlyOnce = true;

    boolean delivering = false;

    boolean gotTwo = false;

    ElapsedTime gripperControl = new ElapsedTime();
    double timeChanger;

    ElapsedTime sweeper = new ElapsedTime();

    double armOffset = 0.08;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        auto = Auto.preload;

        boolean runOnce = false;

        while(!lockIn){

            telemetry.addData("Auto activated", auto);
            telemetry.addData("press y for 2+0", "");
            telemetry.addData("press a for 2+2", "");
            telemetry.addData("press b for 2+4", "");
            telemetry.addData("press x to lock in!!!!", "");
            telemetry.update();

            if (gamepad1.a){
                auto = Auto.two;
            } else if (gamepad1.b) {
                auto = Auto.four;
            } else if (gamepad1.y) {
                auto = Auto.preload;
            }else if (gamepad1.x) {
                lockIn = true;
            }

        }

        telemetry.update();

        waitForStart();

        collection.setIntakeHeight(Collection.intakeHeightState.letClawThrough);
        collection.updateIntakeHeight();

        portal.close();

        if (propPos == 1){

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload);

            while (!(phase == Phase.finished)  && opModeIsActive()){

                switch (phase){
                    case purple:

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 90;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(210 - odometry.X) < 10 && Math.abs(300 - odometry.Y) < 10){
                                targetHeading = 0;
                            }

                            if (Math.abs(241 - odometry.X) < 15 && Math.abs(260 - odometry.Y) < 15 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(300, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }


                        }else if (Math.abs(305 - odometry.X) < 5 && Math.abs(265 - odometry.Y) < 5 && !pathing){

                            if (auto == Red_Right.Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixelWait();
                                phase = Red_Right.Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixel();

                                collect.buildPath(redRightBuilder.Section.collect);

                                deliver.buildPath(redRightBuilder.Section.deliver);

                                phase = Red_Right.Phase.first2;

                                builtPath = false;

                            }
                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition());

                        if (!builtPath){

                            builtPath = true;

                            pathing = true;

                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (!delivering){

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(207 - odometry.Y) < 30){

                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                            }

                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                            delivery.updateGrippers();
                        }

//                        preIntakeCurrentDraw = intakeCurrentDraw;

                        RobotLog.d("Intake current draw" + collection.getIntakeCurrentUse());

//                        intakeCurrentDraw = collection.getIntakeCurrentUse();
//
//                        if (intakeCurrentDraw > 3700 && preIntakeCurrentDraw < 3700 && !reversingIntake){
//                            intakeJammed.reset();
//                        }
//
//                        if (intakeJammed.milliseconds() > 100 && !reversingIntake){
//
//                            reversingIntake = true;
//
//                            preState = collection.getPowerState();
//
//                            collection.setState(Collection.intakePowerState.reversed);
//                            collection.updateIntakeState();
//
//                        }
//
//                        if (intakeJammed.milliseconds() > 250 && reversingIntake){
//
//                            reversingIntake = false;
//
//                            collection.setState(preState);
//                            collection.updateIntakeState();
//
//                        }


                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

                            if (lengthToEnd > 30){
                                timeChanger = 0;
                            } else if (lengthToEnd < 30) {
                                timeChanger = 500;
                            }

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

                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){

                            collection.setState(Collection.intakePowerState.reversedHalf);
                            collection.updateIntakeState();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){

                            collection.setState(Collection.intakePowerState.on);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (delivering) {

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(700, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 220 && sweeper.milliseconds() > 450 && deliverySlides.getCurrentposition() > 100){
                                collection.setSweeperState(Collection.sweeperState.push);
                                collection.updateSweeper();
                                sweeper.reset();
                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();
                            } else if (sweeper.milliseconds() > 400) {

                                collection.setSweeperState(Collection.sweeperState.retract);
                                collection.updateSweeper();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(206 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(197 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(270 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);

                                boolean reachedTarget = false;

                                while (!reachedTarget){
                                    reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
                                    delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);
                                }

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                gotTwo = false;

                                sleep(100);

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Red_Right.Auto.two) {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    retractWait();
                                    phase = Red_Right.Phase.finished;

                                } else {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    builtPath = false;
                                    delivering = false;

                                    retract();
                                    phase = Red_Right.Phase.second2;

                                }
                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 6 && Math.abs(240 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            if (!gotTwo){
                                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
                                collection.updateIntakeHeight();
                            }

                            onlyOnce = false;

                        }

                        break;
                    case second2:

                        delivery.updateArm(deliverySlides.getCurrentposition());

                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (!delivering){

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(207 - odometry.Y) < 30){

                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();
                            }

                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

                            if (lengthToEnd > 30){
                                timeChanger = 0;
                            } else if (lengthToEnd < 30) {
                                timeChanger = 500;
                            }

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

                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){
                            collection.setState(Collection.intakePowerState.reversedHalf);
                            collection.updateIntakeState();
                        }


                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){

                            collection.setState(Collection.intakePowerState.on);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (delivering) {

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(800, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 220 && sweeper.milliseconds() > 450 && deliverySlides.getCurrentposition() > 100){
                                collection.setSweeperState(Collection.sweeperState.push);
                                collection.updateSweeper();
                                sweeper.reset();
                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();
                            } else if (sweeper.milliseconds() > 400 && sweeper.milliseconds() < 450) {
                                collection.setSweeperState(Collection.sweeperState.retract);
                                collection.updateSweeper();
                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(206 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(197 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(270 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);

                                boolean reachedTarget = false;

                                while (!reachedTarget){
                                    reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
                                    delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);
                                }

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                gotTwo = false;

                                sleep(100);

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retractWait();
                                phase = Red_Right.Phase.finished;

                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 5 && Math.abs(240 - odometry.Y) < 5) {

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            if (!gotTwo){
                                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
                                collection.updateIntakeHeight();
                            }

                            onlyOnce = false;

                        }

                        break;
                    default:
                }

            }

        } else if (propPos == 2) {

            firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);

            while (!(phase == Phase.finished) && opModeIsActive()){

                switch (phase){
                    case purple:

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 90;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(214 - odometry.X) < 15 && Math.abs(286 - odometry.Y) < 15){
                                targetHeading = 60;
                            }

                            if (Math.abs(241 - odometry.X) < 15 && Math.abs(274 - odometry.Y) < 15 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(300, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(305 - odometry.X) < 5 && Math.abs(274 - odometry.Y) < 5 && !pathing) {

                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixelWait();

                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                dropYellowPixel();

                                collect.buildPath(redRightBuilder.Section.collect);

                                deliver.buildPath(redRightBuilder.Section.deliver);

                                builtPath = false;

                                phase = Phase.first2;
                            }
                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition());

                        if (!builtPath){

                            builtPath = true;

                            pathing = true;

                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (!delivering){

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(207 - odometry.Y) < 30){

                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                            }

                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                            delivery.updateGrippers();
                        }

//                        preIntakeCurrentDraw = intakeCurrentDraw;

                        RobotLog.d("Intake current draw" + collection.getIntakeCurrentUse());

//                        intakeCurrentDraw = collection.getIntakeCurrentUse();
//
//                        if (intakeCurrentDraw > 3700 && preIntakeCurrentDraw < 3700 && !reversingIntake){
//                            intakeJammed.reset();
//                        }
//
//                        if (intakeJammed.milliseconds() > 100 && !reversingIntake){
//
//                            reversingIntake = true;
//
//                            preState = collection.getPowerState();
//
//                            collection.setState(Collection.intakePowerState.reversed);
//                            collection.updateIntakeState();
//
//                        }
//
//                        if (intakeJammed.milliseconds() > 250 && reversingIntake){
//
//                            reversingIntake = false;
//
//                            collection.setState(preState);
//                            collection.updateIntakeState();
//
//                        }


                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

                            if (lengthToEnd > 30){
                                timeChanger = 0;
                            } else if (lengthToEnd < 30) {
                                timeChanger = 500;
                            }

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

                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){

                            collection.setState(Collection.intakePowerState.reversedHalf);
                            collection.updateIntakeState();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){

                            collection.setState(Collection.intakePowerState.on);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (delivering) {

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(700, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 220 && sweeper.milliseconds() > 450 && deliverySlides.getCurrentposition() > 100){
                                collection.setSweeperState(Collection.sweeperState.push);
                                collection.updateSweeper();
                                sweeper.reset();
                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();
                            } else if (sweeper.milliseconds() > 400) {

                                collection.setSweeperState(Collection.sweeperState.retract);
                                collection.updateSweeper();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(206 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(197 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(270 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);

                                boolean reachedTarget = false;

                                while (!reachedTarget){
                                    reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
                                    delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);
                                }

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                gotTwo = false;

                                sleep(100);

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Red_Right.Auto.two) {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    retractWait();
                                    phase = Red_Right.Phase.finished;

                                } else {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    builtPath = false;
                                    delivering = false;

                                    retract();
                                    phase = Red_Right.Phase.second2;

                                }
                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 6 && Math.abs(240 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            if (!gotTwo){
                                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
                                collection.updateIntakeHeight();
                            }

                            onlyOnce = false;

                        }

                        break;
                    case second2:

                        delivery.updateArm(deliverySlides.getCurrentposition());

                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (!delivering){

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(207 - odometry.Y) < 30){

                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();
                            }

                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

                            if (lengthToEnd > 30){
                                timeChanger = 0;
                            } else if (lengthToEnd < 30) {
                                timeChanger = 500;
                            }

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

                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){
                            collection.setState(Collection.intakePowerState.reversedHalf);
                            collection.updateIntakeState();
                        }


                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){

                            collection.setState(Collection.intakePowerState.on);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (delivering) {

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(800, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 220 && sweeper.milliseconds() > 450 && deliverySlides.getCurrentposition() > 100){
                                collection.setSweeperState(Collection.sweeperState.push);
                                collection.updateSweeper();
                                sweeper.reset();
                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();
                            } else if (sweeper.milliseconds() > 400 && sweeper.milliseconds() < 450) {
                                collection.setSweeperState(Collection.sweeperState.retract);
                                collection.updateSweeper();
                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(206 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(197 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(270 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);

                                boolean reachedTarget = false;

                                while (!reachedTarget){
                                    reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
                                    delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);
                                }

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                gotTwo = false;

                                sleep(100);

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retractWait();
                                phase = Red_Right.Phase.finished;

                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 5 && Math.abs(240 - odometry.Y) < 5) {

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            if (!gotTwo){
                                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
                                collection.updateIntakeHeight();
                            }

                            onlyOnce = false;

                        }

                        break;
                    default:
                }

            }

        } else if (propPos == 3) {

            firstPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.preload);

            while (!(phase == Phase.finished) && opModeIsActive()){

                switch (phase){

                    case purple:

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 90;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(222 - odometry.X) < 8 && Math.abs(308 - odometry.Y) < 10 && !runOnce){
                                runOnce = true;
                                targetHeading = 50;
                            }

                            if (Math.abs(270 - odometry.X) < 7 && Math.abs(285 - odometry.Y) < 10 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(300, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if  (Math.abs(302 - odometry.X) < 5 && Math.abs(285 - odometry.Y) < 5 && !pathing){

                            if (auto == Red_Right.Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixelWait();
                                phase = Red_Right.Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixel();

                                collect.buildPath(redRightBuilder.Section.collect);

                                deliver.buildPath(redRightBuilder.Section.deliver);

                                phase = Red_Right.Phase.first2;

                                builtPath = false;
                            }
                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition());

                        if (!builtPath){

                            builtPath = true;

                            pathing = true;

                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (!delivering){

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(207 - odometry.Y) < 30){

                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                            }

                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                            delivery.updateGrippers();
                        }

//                        preIntakeCurrentDraw = intakeCurrentDraw;

                        RobotLog.d("Intake current draw" + collection.getIntakeCurrentUse());

//                        intakeCurrentDraw = collection.getIntakeCurrentUse();
//
//                        if (intakeCurrentDraw > 3700 && preIntakeCurrentDraw < 3700 && !reversingIntake){
//                            intakeJammed.reset();
//                        }
//
//                        if (intakeJammed.milliseconds() > 100 && !reversingIntake){
//
//                            reversingIntake = true;
//
//                            preState = collection.getPowerState();
//
//                            collection.setState(Collection.intakePowerState.reversed);
//                            collection.updateIntakeState();
//
//                        }
//
//                        if (intakeJammed.milliseconds() > 250 && reversingIntake){
//
//                            reversingIntake = false;
//
//                            collection.setState(preState);
//                            collection.updateIntakeState();
//
//                        }


                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

                            if (lengthToEnd > 30){
                                timeChanger = 0;
                            } else if (lengthToEnd < 30) {
                                timeChanger = 500;
                            }

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

                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){

                            collection.setState(Collection.intakePowerState.reversedHalf);
                            collection.updateIntakeState();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){

                            collection.setState(Collection.intakePowerState.on);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (delivering) {

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(700, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 220 && sweeper.milliseconds() > 450 && deliverySlides.getCurrentposition() > 100){
                                collection.setSweeperState(Collection.sweeperState.push);
                                collection.updateSweeper();
                                sweeper.reset();
                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();
                            } else if (sweeper.milliseconds() > 400) {

                                collection.setSweeperState(Collection.sweeperState.retract);
                                collection.updateSweeper();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(206 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(197 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(270 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);

                                boolean reachedTarget = false;

                                while (!reachedTarget){
                                    reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
                                    delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);
                                }

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                gotTwo = false;

                                sleep(100);

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Red_Right.Auto.two) {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    retractWait();
                                    phase = Red_Right.Phase.finished;

                                } else {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    builtPath = false;
                                    delivering = false;

                                    retract();
                                    phase = Red_Right.Phase.second2;

                                }
                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 6 && Math.abs(240 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            if (!gotTwo){
                                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
                                collection.updateIntakeHeight();
                            }

                            onlyOnce = false;

                        }

                        break;
                    case second2:

                        delivery.updateArm(deliverySlides.getCurrentposition());

                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.firstPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (!delivering){

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(207 - odometry.Y) < 30){

                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();
                            }

                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                            delivery.updateGrippers();
                        }

                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));

                            if (lengthToEnd > 30){
                                timeChanger = 0;
                            } else if (lengthToEnd < 30) {
                                timeChanger = 500;
                            }

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

                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){
                            collection.setState(Collection.intakePowerState.reversedHalf);
                            collection.updateIntakeState();
                        }


                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){

                            collection.setState(Collection.intakePowerState.on);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){

                            collection.setState(Collection.intakePowerState.off);
                            collection.updateIntakeState();

                            delivery.setGripperState(Delivery.GripperState.closed);
                            delivery.updateGrippers();

                        }

                        if (delivering) {

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(800, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 220 && sweeper.milliseconds() > 450 && deliverySlides.getCurrentposition() > 100){
                                collection.setSweeperState(Collection.sweeperState.push);
                                collection.updateSweeper();
                                sweeper.reset();
                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();
                            } else if (sweeper.milliseconds() > 400 && sweeper.milliseconds() < 450) {
                                collection.setSweeperState(Collection.sweeperState.retract);
                                collection.updateSweeper();
                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(206 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(197 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(270 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.setArmTargetState(Delivery.armState.delivery);
                                delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);

                                boolean reachedTarget = false;

                                while (!reachedTarget){
                                    reachedTarget = delivery.getArmState() == Delivery.armState.delivery;
                                    delivery.updateArm(deliverySlides.getCurrentposition(), armOffset);
                                }

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                gotTwo = false;

                                sleep(100);

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retractWait();
                                phase = Red_Right.Phase.finished;

                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 5 && Math.abs(240 - odometry.Y) < 5) {

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            if (!gotTwo){
                                collection.setIntakeHeight(Collection.intakeHeightState.secondPixel);
                                collection.updateIntakeHeight();
                            }

                            onlyOnce = false;

                        }

                        break;
                    default:
                }
            }
        }

    }

    private void initialize(){

        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        init(hardwareMap);

        sensors.init(hardwareMap);

        odometry.update();

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);

    }

}
