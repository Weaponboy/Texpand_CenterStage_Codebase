package org.firstinspires.ftc.teamcode.Auto.Comp_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import static java.lang.Thread.sleep;

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

@Autonomous(name = "Blue_Auto_Left", group = "Newest auto's")
public class Blue_Auto_Left extends LinearOpMode implements CycleMethods {

    /** control point naming key
     * don't need start position because i have sub classes for each one
     * DP = drop purple pixel, DY = drop yellow pixel, C = collect pixels, D = deliver pixels from stack
     * S = start point, C = control point, CT = control point two, E = end point
     * 1 = first segment, 2 = second segment, 3 = third segment
     * F = first prop pos, S = second prop pos, T = third prop pos
     * */

    /**
     * drop preload pixels
     * */
    Vector2D DPS1F = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1F = new Vector2D(getRealCoords(241), getRealCoords(42));
    Vector2D DPCT1F = new Vector2D(getRealCoords(188), getRealCoords(78));
    Vector2D DPE1F = new Vector2D(getRealCoords(295), getRealCoords(95));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(76));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(65));
    Vector2D DPE1S = new Vector2D(getRealCoords(296), getRealCoords(95));

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(223), getRealCoords(79));
    Vector2D DPCT1T = new Vector2D(getRealCoords(164), getRealCoords(110));
    Vector2D DPE1T = new Vector2D(getRealCoords(298), getRealCoords(90));

    Vector2D DS1F = new Vector2D(getRealCoords(300), getRealCoords(135));

    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(292), getRealCoords(154));
    Vector2D CE1F = new Vector2D(getRealCoords(240), getRealCoords(150));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(65), getRealCoords(150));

    Vector2D lastPoint = new Vector2D(getRealCoords(38), getRealCoords(150));

    /**Action points*/
    Vector2D turnIntakeOn = new Vector2D(getRealCoords(183), getRealCoords(153));
    Vector2D turnIntakeOff = new Vector2D(getRealCoords(102), getRealCoords(150));
    Vector2D reverseIntake = new Vector2D(getRealCoords(72), getRealCoords(150));

    GenMethods preloadPaths = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods deliverLast = new GenMethods();

    /**hardware objects*/
    Odometry odometry = new Odometry(210, 23, 270);

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
        four
    }

    enum Build{
        built,
        notBuilt
    }

    Blue_Auto_Left.Phase phase = Phase.preload;

    Blue_Auto_Left.Build build = Build.notBuilt;

    Blue_Auto_Left.Auto auto = Blue_Auto_Left.Auto.preload;

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
                auto = Blue_Auto_Left.Auto.two;
            } else if (gamepad1.b) {
                auto = Blue_Auto_Left.Auto.four;
            } else if (gamepad1.y) {
                auto = Blue_Auto_Left.Auto.preload;
            }else if (gamepad1.x) {
                lockIn = true;
            }

        }

        waitForStart();

        if (propPos == 1){

            preloadPaths.fourPoints(DPS1F, DPC1F, DPCT1F, DPE1F, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Left.Phase.finished)){

                switch (phase){
                    case preload:

//                        delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

                        odometry.update();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(220 - odometry.X) < 10 && Math.abs(65 - odometry.Y) < 10 && !(targetHeading == 310)){
                                targetHeading = 320;
                            }

                            if (Math.abs(260 - odometry.X) < 10 && Math.abs(90 - odometry.Y) < 10 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(350, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition(), odometry, false);

                            }


                        }else if (Math.abs(295 - odometry.X) < 5 && Math.abs(95 - odometry.Y) < 5 && !pathing){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){
                                dropYellowPixelWait(0.61, odometry, telemetry);
                                phase = Phase.finished;
                            }else {

                                dropYellowPixel(0.61, odometry, telemetry);

                                phase = Phase.first2;

                                collect.threePoints(CS1F, CC1F, CE1F);
                                collect.twoPoints(CS2F, CE2F, true);

                                deliver.twoPoints(CE2F, CS2F);
                                deliver.threePoints(CE1F, CC1F, DS1F, true);

                                build = Build.notBuilt;

                            }

                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition());
                        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

                        telemetry.addData("x", odometry.X);
                        telemetry.addData("y", odometry.Y);

                        if (build == Build.notBuilt){
                            follower.setPath(collect.followablePath, collect.pathingVelocity);
                            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
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

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(153 - odometry.Y) < 30){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();
                            }

                        }
//
//                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
//                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
//                            delivery.updateGrippers();
//                        }
//
//                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
//                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
//                            delivery.updateGrippers();
//                        }
//
//                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){
//
//                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));
//
//                            if (lengthToEnd > 30){
//                                timeChanger = 0;
//                            } else if (lengthToEnd < 30) {
//                                timeChanger = 500;
//                            }
//
//                            gripperControl.reset();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                            drive.RF.setPower(0);
//                            drive.RB.setPower(0);
//                            drive.LF.setPower(0);
//                            drive.LB.setPower(0);
//
//                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);
//
//                            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
//
//                            delivering = true;
//
//                            onlyOnce = false;
//
//                            gotTwo = true;
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){
//
//                            collection.setState(Collection.intakePowerState.reversedHalf);
//                            collection.updateIntakeState();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){
//
//                            collection.setState(Collection.intakePowerState.on);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.open);
//                            delivery.updateGrippers();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){
//
//                            collection.setState(Collection.intakePowerState.off);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){
//
//                            collection.setState(Collection.intakePowerState.off);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                        }

                        if (delivering){

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(700, 1);
                                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(150 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(150 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(121 - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Blue_Auto_Left.Auto.two) {

                                    dropWhitePixels(0.64, odometry, telemetry);

                                    while (deliverySlides.getCurrentposition() > 30){

                                    }

                                    phase = Blue_Auto_Left.Phase.finished;

                                } else {

                                    build = Build.notBuilt;

                                    delivering = false;

                                    phase = Blue_Auto_Left.Phase.second2;

                                }
                            }

                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                        }else if (Math.abs(65 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6){

                            deliverLast.twoPoints(new Vector2D(odometry.X, odometry.Y), lastPoint, true);

                            follower.setPath(deliverLast.followablePath, deliverLast.pathingVelocity);

                            pathing = true;

                        } else if (Math.abs(38 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            if (!gotTwo){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.ArmExtension.setPosition(1);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(1000);

                                collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                                collection.updateIntakeHeight();

                                sleep(1000);

                                collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
                                collection.updateIntakeHeight();

                                sleep(1000);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                sleep(400);

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }else {
                                gotTwo = false;
                            }

                            pathing = true;

                            onlyOnce = false;

                            delivering = true;

                        }

                        break;
                    case third2:
                        break;
                    default:
                }
            }

        } else if (propPos == 2) {

            preloadPaths.fourPoints(DPS1S, DPC1S, DPCT1S, DPE1S, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Left.Phase.finished) && opModeIsActive()){

                switch (phase){
                    case preload:

                        odometry.update();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(218 - odometry.X) < 10 && Math.abs(70 - odometry.Y) < 10 && !(targetHeading == 310)){
                                targetHeading = 310;
                            }

                            if (Math.abs(241 - odometry.X) < 12 && Math.abs(86 - odometry.Y) < 12 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(400, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }


                        }else if (Math.abs(DPE1S.getX() - odometry.X) < 5 && Math.abs(DPE1S.getY() - odometry.Y) < 5 && !pathing){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){

                                dropYellowPixelWait(0.5, odometry, telemetry);

                                phase = Phase.finished;

                            }else {

                                dropYellowPixel(0.5, odometry, telemetry);

                                phase = Phase.first2;

                                collect.threePoints(CS1F, CC1F, CE1F);
                                collect.twoPoints(CS2F, CE2F, true);

                                deliver.twoPoints(CE2F, CS2F);
                                deliver.threePoints(CE1F, CC1F, DS1F, true);

                                build = Build.notBuilt;

                            }

                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition());
                        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

                        telemetry.addData("x", odometry.X);
                        telemetry.addData("y", odometry.Y);

                        if (build == Build.notBuilt){

                            follower.setPath(collect.followablePath, collect.pathingVelocity);

                            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

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

                            if (Math.abs(turnIntakeOn.getX() - odometry.X) < 30 && Math.abs(turnIntakeOn.getY() - odometry.Y) < 30){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();
                            }

                        }

//                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
//                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
//                            delivery.updateGrippers();
//                        }
//
//                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
//                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
//                            delivery.updateGrippers();
//                        }
//
//                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){
//
//                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));
//
//                            if (lengthToEnd > 30){
//                                timeChanger = 0;
//                            } else if (lengthToEnd < 30) {
//                                timeChanger = 500;
//                            }
//
//                            gripperControl.reset();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                            drive.RF.setPower(0);
//                            drive.RB.setPower(0);
//                            drive.LF.setPower(0);
//                            drive.LB.setPower(0);
//
//                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);
//
//                            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
//
//                            delivering = true;
//
//                            onlyOnce = false;
//
//                            gotTwo = true;
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){
//
//                            collection.setState(Collection.intakePowerState.reversedHalf);
//                            collection.updateIntakeState();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){
//
//                            collection.setState(Collection.intakePowerState.on);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.open);
//                            delivery.updateGrippers();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){
//
//                            collection.setState(Collection.intakePowerState.off);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){
//
//                            collection.setState(Collection.intakePowerState.off);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                        }
                        if (delivering){

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(700, 1);
                                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(turnIntakeOff.getX() - odometry.X) < 10 && Math.abs(turnIntakeOff.getY() - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(reverseIntake.getX() - odometry.X) < 10 && Math.abs(reverseIntake.getY() - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(DS1F.getX() - odometry.X) < 2 && Math.abs(DS1F.getY() - odometry.Y) < 10) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Blue_Auto_Left.Auto.two) {

                                    dropWhitePixels(0.64, odometry, telemetry);

                                    while (deliverySlides.getCurrentposition() > 30){

                                    }

                                    phase = Blue_Auto_Left.Phase.finished;

                                } else {

                                    build = Build.notBuilt;

                                    delivering = false;

                                    phase = Blue_Auto_Left.Phase.second2;

                                }
                            }

                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                        }else if (Math.abs(CE2F.getX() - odometry.X) < 6 && Math.abs(CE2F.getY() - odometry.Y) < 6){

                            deliverLast.twoPoints(new Vector2D(odometry.X, odometry.Y), lastPoint, true);

                            follower.setPath(deliverLast.followablePath, deliverLast.pathingVelocity);

                            pathing = true;

                        } else if (Math.abs(lastPoint.getX() - odometry.X) < 6 && Math.abs(lastPoint.getY() - odometry.Y) < 6){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            if (!gotTwo){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.ArmExtension.setPosition(1);

                                collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                                collection.updateIntakeHeight();

                                sleep(500);

                                collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
                                collection.updateIntakeHeight();

                                sleep(1000);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                sleep(500);

                            }else {
                                gotTwo = false;
                            }

                            pathing = true;

                            onlyOnce = false;

                            delivering = true;

                        }

                        break;
                    case third2:
                        break;
                    default:
                }
            }

        }else if (propPos == 3) {

            preloadPaths.fourPoints(DPS1T, DPC1T, DPCT1T, DPE1T, true);

            collection.setIntakeHeight(Collection.intakeHeightState.stowed);
            collection.updateIntakeHeight();

            while (!(phase == Blue_Auto_Left.Phase.finished)){

                switch (phase){
                    case preload:

                        odometry.update();

                        if (build == Build.notBuilt){
                            follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                            pathing = true;
                            build = Build.built;
                            targetHeading = 270;
                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                            if (Math.abs(210 - odometry.X) < 10 && Math.abs(55 - odometry.Y) < 10){
                                targetHeading = 0;
                            }

                            if (Math.abs(241 - odometry.X) < 15 && Math.abs(100 - odometry.Y) < 15 && deliverySlides.getCurrentposition() < 50){

                                targetHeading = 180;

                                deliverySlides.DeliverySlides(350, 1);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                        }else if (Math.abs(298 - odometry.X) < 5 && Math.abs(90 - odometry.Y) < 5 && !pathing){

                            drive.RF.setPower(0);
                            drive.RB.setPower(0);
                            drive.LF.setPower(0);
                            drive.LB.setPower(0);

                            if (auto == Auto.preload){

                                dropYellowPixelWait(0.35, odometry, telemetry);
                                phase = Phase.finished;

                            }else {

                                dropYellowPixel(0.35, odometry, telemetry);

                                phase = Phase.first2;

                                collect.threePoints(CS1F, CC1F, CE1F);
                                collect.twoPoints(CS2F, CE2F, true);

                                deliver.twoPoints(CE2F, CS2F);
                                deliver.threePoints(CE1F, CC1F, DS1F, true);

                                build = Build.notBuilt;

                            }

                        }

                        break;
                    case first2:

                        delivery.updateArm(deliverySlides.getCurrentposition());
                        deliverySlides.updateSlides(gamepad1, gamepad2, delivery.getArmState());

                        telemetry.addData("x", odometry.X);
                        telemetry.addData("y", odometry.Y);

                        if (build == Build.notBuilt){
                            follower.setPath(collect.followablePath, collect.pathingVelocity);
                            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
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

                            if (Math.abs(183 - odometry.X) < 30 && Math.abs(153 - odometry.Y) < 30){
                                collection.setState(Collection.intakePowerState.on);
                                collection.updateIntakeState();

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();
                            }

                        }
//
//                        if (!gotTwo && !sensors.RightClawSensor.isPressed()){
//                            delivery.setRightGripperState(Delivery.rightGripperState.closed);
//                            delivery.updateGrippers();
//                        }
//
//                        if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
//                            delivery.setLeftGripperState(Delivery.leftGripperState.closed);
//                            delivery.updateGrippers();
//                        }
//
//                        if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){
//
//                            double lengthToEnd = follower.getSectionLength(new Vector2D(odometry.X, odometry.Y), new Vector2D(44, 264));
//
//                            if (lengthToEnd > 30){
//                                timeChanger = 0;
//                            } else if (lengthToEnd < 30) {
//                                timeChanger = 500;
//                            }
//
//                            gripperControl.reset();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                            drive.RF.setPower(0);
//                            drive.RB.setPower(0);
//                            drive.LF.setPower(0);
//                            drive.LB.setPower(0);
//
//                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);
//
//                            follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));
//
//                            delivering = true;
//
//                            onlyOnce = false;
//
//                            gotTwo = true;
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+100) && gripperControl.milliseconds() < (timeChanger+1000) && gotTwo){
//
//                            collection.setState(Collection.intakePowerState.reversedHalf);
//                            collection.updateIntakeState();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180){
//
//                            collection.setState(Collection.intakePowerState.on);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.open);
//                            delivery.updateGrippers();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1000) && gripperControl.milliseconds() < (timeChanger+1700) && gotTwo && odometry.X < 180 && odometry.X > 170){
//
//                            collection.setState(Collection.intakePowerState.off);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                        }
//
//                        if (gripperControl.milliseconds() > (timeChanger+1700) && gripperControl.milliseconds() < (timeChanger+1800) && gotTwo){
//
//                            collection.setState(Collection.intakePowerState.off);
//                            collection.updateIntakeState();
//
//                            delivery.setGripperState(Delivery.GripperState.closed);
//                            delivery.updateGrippers();
//
//                        }

                        if (delivering){

                            if (odometry.X > 180 && odometry.getVerticalVelocity() > -5 && !onlyOnce) {

                                onlyOnce = true;

                                deliverySlides.DeliverySlides(700, 1);
                                deliverySlides.setSlideState(Delivery_Slides.SlideState.moving);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                            }

                            if (odometry.X > 200 && odometry.getVerticalVelocity() > -5) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                delivery.setArmTargetState(Delivery.armState.deliverAuto);
                                delivery.updateArm(deliverySlides.getCurrentposition());

                            }

                            if (Math.abs(102 - odometry.X) < 10 && Math.abs(150 - odometry.Y) < 10 && !gotTwo) {

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.off);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(72 - odometry.X) < 10 && Math.abs(150 - odometry.Y) < 10 && !gotTwo){

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                collection.setState(Collection.intakePowerState.reversedHalf);
                                collection.updateIntakeState();

                            }

                            if (Math.abs(300 - odometry.X) < 2 && Math.abs(135 - odometry.Y) < 5) {

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                if (auto == Blue_Auto_Left.Auto.two) {

                                    dropWhitePixels(0.64, odometry, telemetry);

                                    while (deliverySlides.getCurrentposition() > 30){

                                    }

                                    phase = Blue_Auto_Left.Phase.finished;

                                } else {

                                    build = Build.notBuilt;

                                    delivering = false;

                                    phase = Blue_Auto_Left.Phase.second2;

                                }
                            }

                        }

                        if (pathing){

                            pathing = follower.followPathAuto(targetHeading, odometry, drive);

                        }else if (Math.abs(65 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6){

                            deliverLast.twoPoints(new Vector2D(odometry.X, odometry.Y), lastPoint, true);

                            follower.setPath(deliverLast.followablePath, deliverLast.pathingVelocity);

                            pathing = true;

                        } else if (Math.abs(38 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6 && !pathing){

                            follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                            if (!gotTwo){

                                drive.RF.setPower(0);
                                drive.RB.setPower(0);
                                drive.LF.setPower(0);
                                drive.LB.setPower(0);

                                delivery.ArmExtension.setPosition(1);

                                delivery.setGripperState(Delivery.GripperState.open);
                                delivery.updateGrippers();

                                sleep(1000);

                                collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
                                collection.updateIntakeHeight();

                                sleep(1000);

                                collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
                                collection.updateIntakeHeight();

                                sleep(1000);

                                delivery.setGripperState(Delivery.GripperState.closed);
                                delivery.updateGrippers();

                                if (!gotTwo && !sensors.RightClawSensor.isPressed()){
                                    delivery.setRightGripperState(Delivery.rightGripperState.closed);
                                    delivery.updateGrippers();
                                }

                                if (!gotTwo && !sensors.LeftClawSensor.isPressed()){
                                    delivery.setLeftGripperState(Delivery.leftGripperState.closed);
                                    delivery.updateGrippers();
                                }

                                if (!gotTwo && !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed() && odometry.X < 180){

                                    delivery.setGripperState(Delivery.GripperState.closed);
                                    delivery.updateGrippers();

                                    drive.RF.setPower(0);
                                    drive.RB.setPower(0);
                                    drive.LF.setPower(0);
                                    drive.LB.setPower(0);

                                    follower.setPath(deliver.followablePath, deliver.pathingVelocity);

                                    follower.resetClosestPoint(new Vector2D(odometry.X, odometry.Y));

                                    collection.setState(Collection.intakePowerState.reversedHalf);
                                    collection.updateIntakeState();

                                    gotTwo = true;

                                }

                                if (!gotTwo){
                                    collection.setIntakeHeight(Collection.intakeHeightState.thirdPixel);
                                    collection.updateIntakeHeight();
                                }

                            }else {
                                gotTwo = false;
                            }

                            pathing = true;

                            onlyOnce = false;

                            delivering = true;

                        }

                        break;
                    case third2:
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
