package org.firstinspires.ftc.teamcode.Auto.Comp_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Stack_2.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueLeftBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Red Right", group = "State")
public class Red_Right extends LinearOpMode implements CycleMethods {

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.red);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 337, 90);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/
    redRightBuilder firstPath = new redRightBuilder();

    redRightBuilder secondPath = new redRightBuilder();

    redRightBuilder collect = new redRightBuilder();

    redRightBuilder deliver = new redRightBuilder();

    redRightBuilder lastToBackboard = new redRightBuilder();

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

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

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

        if (propPos == 3){

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload);

            while (!(phase == Phase.finished)  && opModeIsActive()){

                switch (phase){
                    case purple:

                        telemetry.addData("dropping purple", "");
                        telemetry.update();

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 240;
                            pathing = true;
                        }

                        if (pathing){
                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                        }else {
                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);
                        }

                        break;
                    case yellow:

                        odometry.update();

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                        }else if (Math.abs(300 - odometry.X) < 2 && Math.abs(82.5 - odometry.Y) < 2){
                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixelWait();
                                phase = Phase.finished;

                            }else {
                                dropYellowPixel();

                                collect.buildPath(redRightBuilder.Section.collect);

                                deliver.buildPath(redRightBuilder.Section.deliver);

                                phase = Phase.first2;
                            }
                        }

                        break;
                    case first2:

                        if (!builtPath){
                            builtPath = true;
                            pathing = true;
                            follower.setPath(collect.followablePath, collect.pathingVelocity);
                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();
                            targetHeading = 180;
                        }

                        if (pathing){
                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                            if (Math.abs(125 - odometry.X) < 15 && Math.abs(180 - odometry.Y) < 15){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();
                            }

                            if (Math.abs(121 - odometry.X) < 15 && Math.abs(153 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(86 - odometry.X) < 15 && Math.abs(139 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > 5 && !onlyOnce){

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(400, 0.8);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(38 - odometry.X) < 2 && Math.abs(130 - odometry.Y) < 2){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            onlyOnce = false;

                        }else if (Math.abs(300 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            deployArm();

                            dropPixels();

                            if (auto == Auto.two){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retractWait();
                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retract();
                                phase = Phase.second2;

                            }


                        }

                        break;
                    case second2:

                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                            if (Math.abs(125 - odometry.X) < 15 && Math.abs(180 - odometry.Y) < 15){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();
                            }

                            if (Math.abs(121 - odometry.X) < 15 && Math.abs(153 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(86 - odometry.X) < 15 && Math.abs(139 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > 5 && !onlyOnce){

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(400, 0.8);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(38 - odometry.X) < 2 && Math.abs(130 - odometry.Y) < 2){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            onlyOnce = false;

                        } else if (Math.abs(300 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            deployArm();

                            dropPixels();

                            retractWait();

                            phase = Phase.finished;

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

                        telemetry.addData("dropping purple", "");
                        telemetry.update();

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 270;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(236 - odometry.X) < 15 && Math.abs(60 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                        }else {
                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);
                        }

                        break;

                    case yellow:

                        odometry.update();

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                        }else if (Math.abs(300 - odometry.X) < 2 && Math.abs(97.5 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixelWait();
                                phase = Phase.finished;

                            }else {
                                dropYellowPixel();

                                collect.buildPath(redRightBuilder.Section.collect);

                                deliver.buildPath(redRightBuilder.Section.deliver);

                                phase = Phase.first2;
                            }

                        }

                        break;
                    case first2:

                        if (!builtPath){
                            builtPath = true;
                            pathing = true;
                            follower.setPath(collect.followablePath, collect.pathingVelocity);
                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();
                            targetHeading = 180;
                        }

                        if (pathing){
                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                            if (Math.abs(125 - odometry.X) < 15 && Math.abs(180 - odometry.Y) < 15){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();
                            }

                            if (Math.abs(121 - odometry.X) < 15 && Math.abs(153 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(86 - odometry.X) < 15 && Math.abs(139 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > 5 && !onlyOnce){

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(400, 0.8);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(38 - odometry.X) < 2 && Math.abs(130 - odometry.Y) < 2){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            onlyOnce = false;

                        }else if (Math.abs(300 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            deployArm();

                            dropPixels();

                            if (auto == Auto.two){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retractWait();
                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retract();
                                phase = Phase.second2;

                            }


                        }

                        break;
                    case second2:

                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                            if (Math.abs(125 - odometry.X) < 15 && Math.abs(180 - odometry.Y) < 15){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();
                            }

                            if (Math.abs(121 - odometry.X) < 15 && Math.abs(153 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(86 - odometry.X) < 15 && Math.abs(139 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > 5 && !onlyOnce){

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(400, 0.8);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(38 - odometry.X) < 2 && Math.abs(130 - odometry.Y) < 2){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            onlyOnce = false;

                        } else if (Math.abs(300 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            deployArm();

                            dropPixels();

                            retractWait();

                            phase = Phase.finished;

                        }

                        break;
                    default:
                }

            }

        } else if (propPos == 1) {

            firstPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload, redRightBuilder.pathSplit.first);

            secondPath.buildPath(redRightBuilder.Position.left, redRightBuilder.Section.preload, redRightBuilder.pathSplit.second);

            while (!(phase == Phase.finished) && opModeIsActive()){

                switch (phase){
                    case purple:

                        telemetry.addData("dropping purple", "");
                        telemetry.update();

                        odometry.update();

                        if (!builtPath){
                            builtPath = true;
                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);
                            targetHeading = 270;
                            pathing = true;
                        }

                        if (pathing){
                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(210 - odometry.X) < 15 && Math.abs(70 - odometry.Y) < 15){
                                targetHeading = 0;
                            }

                        }else if (Math.abs(210 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){
                            phase = Phase.yellow;
                            builtPath = false;
                        }

                        break;
                    case yellow:

                        odometry.update();

                        if (!builtPath){
                            builtPath = true;
                            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);
                            targetHeading = 0;
                            pathing = true;
                        }

                        if (pathing){
                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(90 - odometry.Y) < 15){
                                targetHeading = 180;
                            }

                        }else if (Math.abs(300 - odometry.X) < 2 && Math.abs(110 - odometry.Y) < 2){
                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                dropYellowPixelWait();
                                phase = Phase.finished;

                            }else {
                                dropYellowPixel();

                                collect.buildPath(redRightBuilder.Section.collect);

                                deliver.buildPath(redRightBuilder.Section.deliver);

                                phase = Phase.first2;
                            }
                        }

                        break;
                    case first2:

                        if (!builtPath){
                            builtPath = true;
                            pathing = true;
                            follower.setPath(collect.followablePath, collect.pathingVelocity);
                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();
                            targetHeading = 180;
                        }

                        if (pathing){
                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                            if (Math.abs(125 - odometry.X) < 15 && Math.abs(180 - odometry.Y) < 15){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();
                            }

                            if (Math.abs(121 - odometry.X) < 15 && Math.abs(153 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(86 - odometry.X) < 15 && Math.abs(139 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > 5 && !onlyOnce){

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(400, 0.8);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(38 - odometry.X) < 2 && Math.abs(130 - odometry.Y) < 2){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            onlyOnce = false;

                        }else if (Math.abs(300 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            deployArm();

                            dropPixels();

                            if (auto == Auto.two){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retractWait();
                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                retract();
                                phase = Phase.second2;

                            }


                        }

                        break;
                    case second2:

                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            delivery.setGripperState(Delivery.GripperState.open);
                            delivery.updateGrippers();

                            collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(250 - odometry.X) < 15 && Math.abs(46 - odometry.Y) < 15){
                                targetHeading = 180;
                                phase = Phase.yellow;
                            }

                            if (Math.abs(125 - odometry.X) < 15 && Math.abs(180 - odometry.Y) < 15){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();
                            }

                            if (Math.abs(121 - odometry.X) < 15 && Math.abs(153 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(86 - odometry.X) < 15 && Math.abs(139 - odometry.Y) < 15){

                                collection.setState(Collection.intakePowerState.reversed);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > 5 && !onlyOnce){

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(400, 0.8);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(38 - odometry.X) < 2 && Math.abs(130 - odometry.Y) < 2){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            onlyOnce = false;

                        } else if (Math.abs(300 - odometry.X) < 2 && Math.abs(90 - odometry.Y) < 2){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            deployArm();

                            dropPixels();

                            retractWait();

                            phase = Phase.finished;

                        }

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

        odometry.update();

        frontCam = hardwareMap.get(WebcamName.class, "frontcam");

        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);

    }

}
