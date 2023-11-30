package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintTwo;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.horizontal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.vertical;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getDriveToBackboardControlPoint;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.Auto_Slide_Height;
import org.firstinspires.ftc.teamcode.hardware.Odometry.ObjectAvoidance.Vector2D;
import org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.PathGeneration.TargetPoint;
import org.firstinspires.ftc.teamcode.hardware.Odometry.Pathing.PathGeneration.pathBuilder;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Odometry;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.List;

@TeleOp
public class Testing_Different_Teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Auto_Slide_Height slideHeight;
    private VisionPortal visionPortal;
    WebcamName webcamName;

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    Delivery delivery = new Delivery();

    Odometry odometry = new Odometry(296, 70, 180);

    PIDFController Slide_Power;

    Servo DepositServoRotate;
    Servo clawRight;
    Servo clawLeft;
    Servo DepositPivot;
    Servo TestFeetech;
    Servo IntakeServo;

    int SlidesTarget = 0;

    double SlidePower;

    int Pivot_Target = 0;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    double throttle = 0.6;

    boolean SlideSafetyHeight = false;

    boolean SlideSafetyBottom = false;

    boolean reverseIntake = false;
    double SlideMinDeposit = -900;
    boolean autodeposit = false;
    int RightPixelColThresh = 700;
    double IntakeServopos = 0.3;
    int buttondelaytime = 300;

    ElapsedTime runtime = new ElapsedTime();

    boolean pivotDelivery = false;
    boolean pivotCollection = false;
    boolean pivotMoving = false;

    boolean pivotSafePoint = false;
    boolean slidesCollect = false;
    boolean slidesMoving = false;
    boolean slidesMovingUp = false;
    boolean slidesMovingDown = false;
    boolean retracting = false;
    boolean extending = false;
    boolean Driving = false;
    double pivotPower = 0;

    pathBuilder path = new pathBuilder();

    Vector2D robotPos = new Vector2D();

    mecanumFollower follower = new mecanumFollower();

    boolean pathing = false;

    double targetHeading;

    @Override
    public void loop() {

        odometry.update();

        robotPos.set(odometry.X, odometry.Y);

        /**Drive code*/
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        if(gamepad1.right_stick_button && gamepad1.left_stick_button){
            pathing = false;
        }

        if (pathing && gamepad1.atRest()){
            follower.followPathTeleop(true, targetHeading, false, odometry, drive);
        }else {

            horizontal = -gamepad1.right_stick_x;
            vertical = -gamepad1.right_stick_y;
            pivot = gamepad1.left_stick_x;

            double denominator = Math.max(Math.abs(horizontal) + Math.abs(vertical) + Math.abs(pivot), 1);

            drive.RF.setPower((-pivot + (vertical - horizontal)) / denominator);
            drive.RB.setPower((-pivot + (vertical + horizontal)) / denominator);
            drive.LF.setPower((pivot + (vertical + horizontal)) / denominator);
            drive.LB.setPower((pivot + (vertical - horizontal)) / denominator);

        }

        if (gamepad1.b){

            if (getDriveToBackboardControlPoint(robotPos) != null){
                path.buildPath(TargetPoint.blueBackBoard, robotPos, getDriveToBackboardControlPoint(robotPos));
            }else {
                path.buildPath(TargetPoint.blueBackBoard, robotPos);
            }

            targetHeading = 180;

            pathing = true;

            follower.setPath(path.followablePath, path.pathingVelocity);
        }

        /**Delivery Slide Code*/

        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() < -2200;
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() > -5;

        if (gamepad1.x && !SlideSafetyHeight) {
            SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() < -2200;
            deliverySlides.SlidesBothPower(-0.3);
            slidesMovingUp = true;
            slidesMovingDown = false;
            slidesMoving = true;
        } else if (gamepad1.a && !SlideSafetyBottom) {
            SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() > -5;
            deliverySlides.SlidesBothPower(0.3);
            slidesMovingUp = false;
            slidesMovingDown = true;
            slidesMoving = true;
        } else if (deliverySlides.Left_Slide.getCurrentPosition() > -25){
            deliverySlides.SlidesBothPower(0);
        } else{
            deliverySlides.SlidesBothPower(0.0005);
        }

        if (Math.abs(deliverySlides.Left_Slide.getVelocity()) < 20) {
            slidesMoving = false;
        }

        /**Deposit Code*/

        if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && clawLeft.getPosition() < 0.1) {
            clawLeft.setPosition(0.6);
        } else if (currentGamepad1.dpad_right && !previousGamepad1.dpad_right && clawLeft.getPosition() > 0.1) {
            clawLeft.setPosition(0);
        }

        if (gamepad1.start) {
            clawLeft.setPosition(0.6);
            clawRight.setPosition(0.4);
        }

        if (gamepad1.back) {
            clawLeft.setPosition(0);
            clawRight.setPosition(1);
        }

        if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && clawRight.getPosition() > 0.9) {
            clawRight.setPosition(0.4);
        } else if (currentGamepad1.dpad_left && !previousGamepad1.dpad_left && clawRight.getPosition() < 0.9) {
            clawRight.setPosition(1);
        }

        /**Intake Servo Code*/

//        if (gamepad1.dpad_right && IntakeServo.getPosition() >= 0) {
//            IntakeServo.setPosition(0.5);
//        } else if (gamepad1.dpad_right && IntakeServo.getPosition() <= 0.5) {
//            IntakeServo.setPosition(0);
//        }

        /**Top Arm Control Code*/

        /**Arm Control While Slides Moving*/
        if (slidesMoving && delivery.Pivot.getCurrentPosition() > -100) {

            if (clawLeft.getPosition() > 0.1 || clawRight.getPosition() < 0.9) {
                clawLeft.setPosition(0);
                clawRight.setPosition(1);

                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            } else {
                clawLeft.setPosition(0);
                clawRight.setPosition(1);
            }

            Pivot_Target = 100;
            pivotPower = 0.5;

            Top_Pivot_Position(pivotPower);

            if (deliverySlides.Left_Slide.getCurrentPosition() < -600) {
                Pivot_Target = 0;
                DepositServoRotate.setPosition(1);
                DepositPivot.setPosition(0);
            } else {
                DepositServoRotate.setPosition(0.5);
                DepositPivot.setPosition(0.9);
            }

        }

        if ((Math.abs(drive.RF.getPower()) > 0.15 || Math.abs(drive.RF.getPower()) > 0.15)) {
            Driving = true;
            runtime.reset();
        } else if ((Math.abs(drive.RF.getPower()) > 0.1 || Math.abs(drive.RF.getPower()) > 0.1) && collection.Intake.getPower() == 0 && delivery.Pivot.getCurrentPosition() > -100 && deliverySlides.Left_Slide.getCurrentPosition() > -10) {
            clawLeft.setPosition(0);
            clawRight.setPosition(1);
        } else if ((runtime.milliseconds()) > buttondelaytime){
            Driving = false;
            runtime.reset();
        }

        if( Driving && delivery.Pivot.getCurrentPosition() > -100 && deliverySlides.Left_Slide.getCurrentPosition() > -10 && collection.Intake.getPower() == 0){
            clawLeft.setPosition(0);
            clawRight.setPosition(1);

            try {
                Thread.sleep(50);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
            Pivot_Target = 200;
            pivotPower = 0.9;
            Top_Pivot_Position(pivotPower);


        }else if(delivery.Pivot.getCurrentPosition() > -100 && deliverySlides.Left_Slide.getCurrentPosition() > -10){
            Pivot_Target = 0;
            pivotPower = 0.4;
            Top_Pivot_Position(pivotPower);
            DepositServoRotate.setPosition(0.5);
            DepositPivot.setPosition(0.9);
        }

        if(gamepad1.dpad_up){

            if (clawLeft.getPosition() > 0.1 || clawRight.getPosition() < 0.9) {
                clawLeft.setPosition(0);
                clawRight.setPosition(1);

                try {
                    Thread.sleep(400);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }else{
                clawLeft.setPosition(0);
                clawRight.setPosition(1);
            }
            DepositServoRotate.setPosition(1);

            DepositPivot.setPosition(0);

            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }

            Pivot_Target = 120;

            Top_Pivot_Position(pivotPower);

        }

        if (gamepad1.dpad_down && deliverySlides.Left_Slide.getCurrentPosition() < 590){

            DepositServoRotate.setPosition(1);

            if (clawLeft.getPosition() > 0.1 || clawRight.getPosition() < 0.9) {
                clawLeft.setPosition(0);
                clawRight.setPosition(1);

                try {
                    Thread.sleep(300);
                } catch (InterruptedException e) {
                    throw new RuntimeException(e);
                }
            }else{
                clawLeft.setPosition(0);
                clawRight.setPosition(1);
            }

            Pivot_Target = -990;

            while (delivery.Pivot.getCurrentPosition() > -200){
                Top_Pivot_Position(pivotPower);
            }

            DepositPivot.setPosition(0.5);

            DepositServoRotate.setPosition(0.5);
        }

        /**Intake Toggle*/

        if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper && collection.Intake.getPower() == 0) {
            collection.Intake.setPower(1);
        } else if (currentGamepad1.right_bumper && !previousGamepad1.right_bumper) {
            collection.Intake.setPower(0);
        }

        if (currentGamepad1.y && !previousGamepad1.y){
            collection.Intake.setPower(-1);
        }

        /**Call all PID's and telemetry code*/

        Top_Pivot_Position(pivotPower);

        TelemetryMap();

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.init(hardwareMap);

        odometry.init(hardwareMap);

        robotPos.set(odometry.X, odometry.Y);

        DepositServoRotate = hardwareMap.get(Servo.class, "DepositRotate");
        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
        clawRight = hardwareMap.get(Servo.class, "clawRight");
        DepositPivot = hardwareMap.get(Servo.class, "DepositPivot");
        TestFeetech = hardwareMap.get(Servo.class, "fitec");
        IntakeServo = hardwareMap.get(Servo.class, "intake_servo");

        webcamName = hardwareMap.get(WebcamName.class, "frontCam");

        slideHeight = new Auto_Slide_Height(telemetry);
        visionPortal = VisionPortal.easyCreateWithDefaults(webcamName, slideHeight);

        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);

        currentGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        runtime.reset();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

        clawLeft.setPosition(0.6);
        clawRight.setPosition(0.4);
        DepositServoRotate.setPosition(0.5);
        DepositPivot.setPosition(1);

        TestFeetech.setPosition(0);

        IntakeServo.setPosition(0);

    }

    public void Top_Pivot_Position(double power){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * power;

        delivery.Pivot.setPower(Top_Pivot_PID);

    }

    public void Top_Pivot_Position_With_Feedforward(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.2;

        double pivot_f = 0.1;

        double ticks_in_degrees = 2550 / 180.0;

        double Pivot_FF = Math.cos(Math.toRadians(Pivot_Target / ticks_in_degrees)) * pivot_f;

        double Pivot_Power = Top_Pivot_PID + Pivot_FF;

        delivery.Pivot.setPower(Pivot_Power);

    }

    public void CheckSlidePos(){
        SlidePower = Slide_Power.calculate(deliverySlides.Left_Slide.getCurrentPosition(), SlidesTarget);
        deliverySlides.SlidesBothPower(SlidePower);
    }

    public void calculateSlidePowerUp(){
        SlidesTarget = deliverySlides.Left_Slide.getCurrentPosition() + 30;
        SlidePower = Slide_Power.calculate(SlidesTarget);
        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() > 2200;
        deliverySlides.SlidesBothPower(SlidePower);
    }

    public void calculateSlidePowerDown(){
        SlidesTarget = deliverySlides.Left_Slide.getCurrentPosition() - 30;
        SlidePower = Slide_Power.calculate(SlidesTarget);
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() < 10;
        deliverySlides.SlidesBothPower(SlidePower);
    }

    void TelemetryMap(){
        telemetry.addData("left slide", deliverySlides.Left_Slide.getCurrentPosition());
        telemetry.addData("right slide", deliverySlides.Right_Slide.getCurrentPosition());
        telemetry.addData("Deposit Rotate", delivery.Pivot.getVelocity());
        telemetry.addData("pivot pos", delivery.Pivot.getCurrentPosition());
        telemetry.addData("pivot target", Pivot_Target);
        telemetry.addData("Driving", Driving);
        telemetry.update();
    }

}
