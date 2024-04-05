package org.firstinspires.ftc.teamcode.Testing.Test_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.Auto.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueLeftBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
import org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

@Autonomous(name = "Blue_Left_Pathing", group = "Pathing")
public class Blue_Left_Pathing extends LinearOpMode implements CycleMethods{

    public WebcamName frontCam;

    public VisionPortal portal;

    org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.Vision.VisionPortalProcessers.propDetectionByAmount.color.blue);

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 23, 270);

    Drivetrain drive = new Drivetrain();

    /**pathing objects*/

    blueLeftBuilder firstPath = new blueLeftBuilder();

    blueLeftBuilder secondPath = new blueLeftBuilder();

    blueLeftBuilder collect = new blueLeftBuilder();

    blueLeftBuilder deliver = new blueLeftBuilder();

    blueRightBuilder lastToBackboard = new blueRightBuilder();

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

            firstPath.buildPath(blueLeftBuilder.Position.left, blueLeftBuilder.Section.preload);

            while (!(phase == Phase.finished)  && opModeIsActive()){

                switch (phase){
                    case purple:

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 270;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(218 - odometry.X) < 8 && Math.abs(62 - odometry.Y) < 10 && !runOnce){
                                runOnce = true;
                                targetHeading = 310;
                            }

                            if (Math.abs(270 - odometry.X) < 5 && Math.abs(80 - odometry.Y) < 5 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                            }

                        }else if  (Math.abs(305 - odometry.X) < 5 && Math.abs(82.5 - odometry.Y) < 5 && !pathing){

                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                collect.buildPath(blueLeftBuilder.Section.collect);

                                deliver.buildPath(blueLeftBuilder.Section.deliver);

                                phase = Phase.first2;

                                builtPath = false;
                            }
                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);


                        if (!builtPath){

                            builtPath = true;

                            pathing = true;

                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (delivering) {

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(100 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Blue_Left_Pathing.Auto.two) {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    phase = Blue_Left_Pathing.Phase.finished;

                                } else {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    builtPath = false;
                                    delivering = false;

                                    phase = Blue_Left_Pathing.Phase.second2;

                                }
                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            onlyOnce = false;

                        }

                        break;
                    case second2:

                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);


                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (delivering) {

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(100 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                phase = Blue_Left_Pathing.Phase.finished;

                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(42 - odometry.X) < 5 && Math.abs(150 - odometry.Y) < 5) {

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            onlyOnce = false;

                        }

                        break;
                    default:
                }

            }

        } else if (propPos == 2) {

            firstPath.buildPath(blueLeftBuilder.Position.center, blueLeftBuilder.Section.preload);

            while (!(phase == Phase.finished) && opModeIsActive()){

                switch (phase){
                    case purple:

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 270;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(214 - odometry.X) < 15 && Math.abs(74 - odometry.Y) < 15){
                                targetHeading = 300;
                            }

                            if (Math.abs(241 - odometry.X) < 20 && Math.abs(86 - odometry.Y) < 20 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                            }

                        }else if (Math.abs(305 - odometry.X) < 5 && Math.abs(94 - odometry.Y) < 5 && !pathing) {

                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                collect.buildPath(blueLeftBuilder.Section.collect);

                                deliver.buildPath(blueLeftBuilder.Section.deliver);

                                builtPath = false;

                                phase = Phase.first2;
                            }
                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);


                        if (!builtPath){

                            builtPath = true;

                            pathing = true;

                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (delivering) {

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(121 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Blue_Left_Pathing.Auto.two) {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    phase = Blue_Left_Pathing.Phase.finished;

                                } else {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    builtPath = false;
                                    delivering = false;

                                    phase = Blue_Left_Pathing.Phase.second2;

                                }
                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(50 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            onlyOnce = false;

                        }

                        break;
                    case second2:

                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);


                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (delivering) {

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(120 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                phase = Blue_Left_Pathing.Phase.finished;

                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(50 - odometry.X) < 5 && Math.abs(150 - odometry.Y) < 5) {

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            onlyOnce = false;

                        }

                        break;
                    default:
                }

            }

        } else if (propPos == 3) {

            firstPath.buildPath(blueLeftBuilder.Position.right, blueLeftBuilder.Section.preload, redRightBuilder.pathSplit.first);

            while (!(phase == Phase.finished) && opModeIsActive()){

                switch (phase){
                    case purple:

                        odometry.update();

                        if (!builtPath){

                            builtPath = true;

                            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);

                            targetHeading = 270;

                            pathing = true;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(210 - odometry.X) < 10 && Math.abs(60 - odometry.Y) < 10){
                                targetHeading = 0;
                            }

                            if (Math.abs(241 - odometry.X) < 15 && Math.abs(100 - odometry.Y) < 15 && deliverySlides.getCurrentposition() < 50){
                                targetHeading = 180;
                            }


                        }else if (Math.abs(305 - odometry.X) < 5 && Math.abs(105 - odometry.Y) < 5 && !pathing){

                            if (auto == Auto.preload){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                phase = Phase.finished;

                            }else {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                collect.buildPath(blueLeftBuilder.Section.collect);

                                deliver.buildPath(blueLeftBuilder.Section.deliver);

                                phase = Phase.first2;

                                builtPath = false;

                            }
                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);


                        if (!builtPath){

                            builtPath = true;

                            pathing = true;

                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (delivering) {

                            if (Math.abs(240 - odometry.X) < 10 && Math.abs(151 - odometry.Y) < 10){
                                targetHeading = 165;
                            }

                            if (Math.abs(300 - odometry.X) < 4 && Math.abs(121 - odometry.Y) < 4) {

                                targetHeading = 0;

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Blue_Left_Pathing.Auto.two) {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    phase = Blue_Left_Pathing.Phase.finished;

                                } else {

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    builtPath = false;
                                    delivering = false;

                                    phase = Blue_Left_Pathing.Phase.second2;

                                }
                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(50 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

                            onlyOnce = false;

                        }

                        break;
                    case second2:

                        delivery.updateArm(deliverySlides.getCurrentposition(), false, Delivery.PixelsAuto.backboardRight, odometry);


                        if (!builtPath){

                            builtPath = true;
                            pathing = true;
                            gotTwo = false;

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                            collection.updateIntakeHeight();

                            targetHeading = 180;
                        }

                        odometry.update();

                        if (delivering) {

                            if (Math.abs(240 - odometry.X) < 10 && Math.abs(151 - odometry.Y) < 10){
                                targetHeading = 165;
                            }

                            if (Math.abs(300 - odometry.X) < 4 && Math.abs(121 - odometry.Y) < 4) {

                                targetHeading = 0;

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                phase = Blue_Left_Pathing.Phase.finished;

                            }
                        }

                        if (pathing){

                            if (delivering){
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 2);
                            }else {
                                pathing = follower.followPathAuto(targetHeading, odometry, drive, 5);
                            }

                        }else if (Math.abs(50 - odometry.X) < 5 && Math.abs(150 - odometry.Y) < 5) {

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            pathing = true;

                            delivering = true;

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
