package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;


@TeleOp
public class Sprint_3_teleop extends OpMode {
    public BNO055IMU imu = null;
    Drivetrain drive = new Drivetrain();

    //Leave at 0 start for now
    Odometry odometry = new Odometry();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    DistanceSensor RightClawSensor;
    DistanceSensor LeftClawSensor;

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    //in ms
    double timePerDegreeTopPivot = 3;

    double smallServoTimePerDegree = 3;

    //!!!!!
    //tune these values
    //!!!!

    double collectTopPivotPos = 0.2;
    double deliveryTopPivot = 0.7;
    double lowdeliveryTopPivot = 1;
    double safeTopPivot = 0.3;
    static final double servoPosPerTick = 0.00004100;

    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 1;
    double deliverySecondPivot = 0.13 ;
    double lowdeliverySecondPivot = 0.55;

    static final double mainToSecondConst = 0.5/0.3;

    double clawOpen = 0.25;
    double clawClosed = 0;

    double rotateCollect = 0.5;
    double rotateRight = 1;

    double timeToWait;
    double targetRotatePos;
    static final double adjustFactor = 1.1;
    boolean closeToCollection;
    double mainPivotOffSet = 0;
    double targetMainPivot = 0;
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
            collection.IntakeHeight.setPosition(0.0);
        } else if (currentGamepad1.left_bumper && !previousGamepad1.left_bumper && collection.IntakeHeight.getPosition() < 0.5) {
            collection.IntakeHeight.setPosition(0.7);
        }

        if (currentGamepad1.right_trigger > 0 && !(previousGamepad1.right_trigger > 0)){
            pivotIntakePos ++;

            if (pivotIntakePos == 5){
                pivotIntakePos = 0;
            }
            if(pivotIntakePos == 0){
                collection.IntakeHeight.setPosition(0.275);
            }
            else if(pivotIntakePos == 1){
                collection.IntakeHeight.setPosition(0.25);
            }
            else if(pivotIntakePos == 2){
                collection.IntakeHeight.setPosition(0.20);
            }
            else if(pivotIntakePos == 3){
                collection.IntakeHeight.setPosition(0.175);
            }
            else if(pivotIntakePos == 4){
                collection.IntakeHeight.setPosition(0);
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
                if (gamepad1.dpad_down && deliverySlides.getCurrentposition() > 100) {

                    pivotState = PivotState.transitioning;

                    pivotMoveTime.reset();

                    timeToWait = Math.max((Math.abs(delivery.getSecondPivotPosition() - collectSecondPivot) * 180) * timePerDegreeTopPivot, (Math.abs(delivery.getTopPivotPosition() - collectTopPivotPos) * 180) * timePerDegreeTopPivot);

                    delivery.setClaws(clawClosed);

                    delivery.setSecondPivot(collectSecondPivot);

                    delivery.setMainPivot(collectTopPivotPos);

                    delivery.RotateClaw.setPosition(rotateCollect);

                }
                if(deliverySlides.getCurrentposition() > 200) {

                    if (gamepad1.dpad_right && delivery.mainPivotRight.getPosition() < lowdeliveryTopPivot) {
                        mainPivotOffSet = mainPivotOffSet + 0.005;
//                    delivery.setMainPivot(delivery.mainPivotRight.getPosition() + 0.005);
//                    delivery.setSecondPivot(delivery.pivot1.getPosition() + 0.005 * mainToSecondConst);
                    }
                    if (gamepad1.dpad_left && delivery.mainPivotRight.getPosition() > deliveryTopPivot) {
                        mainPivotOffSet = mainPivotOffSet - 0.005;
//                    delivery.setMainPivot(delivery.mainPivotRight.getPosition() - 0.005);
//                    delivery.setSecondPivot(delivery.pivot1.getPosition() - 0.005 * mainToSecondConst);
                    }

                    targetMainPivot = deliveryTopPivot - deliverySlides.getCurrentposition() * servoPosPerTick + mainPivotOffSet;
                    delivery.setMainPivot(targetMainPivot);
                    delivery.setSecondPivot(deliverySecondPivot + (-deliverySlides.getCurrentposition() * servoPosPerTick + mainPivotOffSet) * mainToSecondConst);

                    targetRotatePos = 0.5 - (GetHeading() / 180) * adjustFactor;
                    delivery.secondRotate.setPosition(targetRotatePos);
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

//        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && delivery.LeftClaw.getPosition() == clawClosed) {
//            delivery.LeftClaw.setPosition(clawOpen);
//        } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && delivery.LeftClaw.getPosition() == clawOpen) {
//            delivery.LeftClaw.setPosition(clawClosed);
//        }

        if (gamepad1.start) {
            delivery.RightClaw.setPosition(clawClosed);
            delivery.LeftClaw.setPosition(clawClosed);
        }

        if (gamepad1.back) {
            delivery.RightClaw.setPosition(clawOpen);
            delivery.LeftClaw.setPosition(clawOpen);
        }

//        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && delivery.RightClaw.getPosition() == clawClosed) {
//            delivery.RightClaw.setPosition(clawOpen);
//        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && delivery.RightClaw.getPosition() == clawOpen) {
//            delivery.RightClaw.setPosition(clawClosed);
//        }
        if(collection.Intake.getPower() > 0){
            if(LeftClawSensor.getDistance(DistanceUnit.CM) < 7){
                delivery.LeftClaw.setPosition(clawClosed);
            }
            if(RightClawSensor.getDistance(DistanceUnit.CM) < 7){
                delivery.RightClaw.setPosition(clawClosed);
            }
        }

//        if (RightClaw.getDistance(DistanceUnit.MM) < 60 && intakePos){
//            delivery.RightClaw.setPosition(clawClosed);
//        }
//
//        if (LeftClaw.getDistance(DistanceUnit.MM) < 60 && intakePos){
//            delivery.LeftClaw.setPosition(clawClosed);
//        }


        telemetry.addData("Slide height", deliverySlides.getCurrentposition());
        telemetry.addData("Base Pivot", delivery.mainPivotLeft.getPosition());
        telemetry.addData("Second Pivot", delivery.pivot1.getPosition());
        telemetry.addData("Intake Servo Position: ", pivotIntakePos);
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

        delivery.setMainPivot(0);

        delivery.setSecondPivot(1);

        delivery.RotateClaw.setPosition(0.5);

        delivery.RightClaw.setPosition(clawClosed);
        delivery.LeftClaw.setPosition(clawClosed);

        collection.IntakeHeight.setPosition(0);

        RightClawSensor = hardwareMap.get(DistanceSensor.class, "rightclaw");
        LeftClawSensor = hardwareMap.get(DistanceSensor.class, "leftclaw");

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        parameters.loggingEnabled = true;
        parameters.loggingTag = "IMU";

        imu = hardwareMap.get(BNO055IMU.class, "imu");

        imu.initialize(parameters);

    }

    private double GetHeading(){
        double ConvertedHeading;

        double heading = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES).firstAngle;

        if (heading <= 0) {
            ConvertedHeading = (360 + heading);
            ConvertedHeading = (360-ConvertedHeading);
        } else {
            ConvertedHeading = (0 + heading);
            ConvertedHeading = (360-ConvertedHeading);
        }
        return heading;
    }
}
