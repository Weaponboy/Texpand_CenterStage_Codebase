package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintTwo;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.currentGamepad1;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.previousGamepad1;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.hardware.SubSystems.Collection;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Delivery;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Drivetrain;

import java.util.List;

@Disabled
public class Sprint_2_teleop extends OpMode {

    Drivetrain drive = new Drivetrain();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    Delivery delivery = new Delivery();

    PIDFController Slide_Power;

    Servo LeftClaw;

    Servo RightClaw;
    Servo Intake_Servo;
    Servo DepositRotate;

    DistanceSensor left_Pixel;
    ColorSensor right_Pixel;

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
    boolean pivottareget = false;
    ElapsedTime runtime = new ElapsedTime();

    @Override
    public void loop() {

        /**Drive code*/
        previousGamepad1.copy(currentGamepad1);

        currentGamepad1.copy(gamepad1);

        throttle = 0.6;

        throttle = (gamepad1.left_trigger * 0.4) + throttle;

        if (gamepad1.right_bumper) {
            throttle = 0.3;
        }

        double vertical = -gamepad1.right_stick_y;
        double horizontal = gamepad1.right_stick_x * 1.5;
        double pivot = gamepad1.left_stick_x;

        drive.RF.setPower(throttle * (-pivot + (vertical - horizontal)));
        drive.RB.setPower((throttle * 1.15) * (-pivot + (vertical + horizontal)));
        drive.LF.setPower(throttle * (pivot + (vertical + horizontal)));
        drive.LB.setPower((throttle * 1.15) * (pivot + (vertical - horizontal)));

        /**Delivery Slide Code*/

        SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() < -2200;
        SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() > -5;


        if (gamepad1.x && !SlideSafetyHeight) {
            SlideSafetyHeight = deliverySlides.Left_Slide.getCurrentPosition() < -2200;
            deliverySlides.SlidesBothPower(-0.3);
            if (Pivot_Target == 0){
                Pivot_Target = 100;
            }
        } else if (gamepad1.a && !SlideSafetyBottom) {
            SlideSafetyBottom = deliverySlides.Left_Slide.getCurrentPosition() > -5;
            deliverySlides.SlidesBothPower(0.3);
            if (Pivot_Target == 0){
                Pivot_Target = 100;
            }
        }
        else {
            deliverySlides.SlidesBothPower(0.0005);

        }

        /**Top pivot code*/


        if ((drive.RF.getPower() > 0.2 || drive.RB.getPower() > 0.2) && delivery.Pivot.getCurrentPosition() < -1300){
            if(deliverySlides.Left_Slide.getCurrentPosition() < SlideMinDeposit) {
                Pivot_Target = -400;
            }else{
                Pivot_Target = -900;
            }

        }


        if (gamepad1.dpad_up && deliverySlides.Left_Slide.getCurrentPosition() < SlideMinDeposit) {
            Pivot_Target = -1400;
            deliverySlides.DeliverySlides(-900,-0.3);
        }

        if (gamepad1.left_bumper && deliverySlides.Left_Slide.getCurrentPosition() < SlideMinDeposit) {
            DepositRotate.setPosition(0);
            Pivot_Target = 100;
            deliverySlides.DeliverySlides(-10,0.3);
        }

        if (gamepad1.dpad_down && deliverySlides.Left_Slide.getCurrentPosition() < SlideMinDeposit) {
            DepositRotate.setPosition(0);
            Pivot_Target = 0;
        }



//        /**Pixel Sensor Conditions*/
//
//        if (left_Pixel.getDistance(DistanceUnit.MM) < 23 && autodeposit) {
//            LeftClaw.setPosition(0.5);                     //Lol I think I did the hardware wrong
//        }
//
//        if ((right_Pixel.blue() > RightPixelColThresh || right_Pixel.red() > RightPixelColThresh || right_Pixel.green() > RightPixelColThresh) && autodeposit) {
//            RightClaw.setPosition(0.5);                        //Lol I think I did the hardware wrong
//        }

        /**Intake Toggle*/

//        if (collection.Intake.getPower() < 0 && left_Pixel.getDistance(DistanceUnit.MM) < 23 && (right_Pixel.blue() > RightPixelColThresh || right_Pixel.red() > RightPixelColThresh || right_Pixel.green() > RightPixelColThresh)) {
//            collection.Intake.setPower(0.4);
//            reverseIntake = true;
//            runtime.reset();
//        }

        if (reverseIntake && (runtime.seconds() > 3)) {
            reverseIntake = false;
            collection.Intake.setPower(0);
        }

        if (gamepad1.right_bumper && collection.Intake.getPower() == 0 && (runtime.milliseconds()) > buttondelaytime) {
            collection.Intake.setPower(1);
            if (delivery.Pivot.getCurrentPosition() > -10) {
                Pivot_Target = 100;
            }
            autodeposit = true;
            runtime.reset();
        } else if (gamepad1.right_bumper && collection.Intake.getPower() > 0 && (runtime.milliseconds()) > buttondelaytime) {
            collection.Intake.setPower(0);
            autodeposit = false;
            runtime.reset();
        }

        /**Deposit Code*/

        //Pick up pixels from intake
        if (gamepad1.left_trigger > 0.1 && delivery.Pivot.getCurrentPosition() > 0){
            Pivot_Target = 0;
            RightClaw.setPosition(0.5);
            LeftClaw.setPosition(0.5);
            autodeposit = false;
        }

        if (gamepad1.dpad_left && LeftClaw.getPosition() < 1 && (runtime.milliseconds()) > buttondelaytime) {
            LeftClaw.setPosition(1);
            autodeposit = false;
            runtime.reset();
        } else if (gamepad1.dpad_left && LeftClaw.getPosition() > 0.5 && (runtime.milliseconds()) > buttondelaytime) {
            LeftClaw.setPosition(0.5);
            autodeposit = false;
            runtime.reset();
        }

        if (gamepad1.dpad_right && RightClaw.getPosition() > 0.1 && (runtime.milliseconds()) > buttondelaytime) {
            RightClaw.setPosition(0);
            autodeposit = false;
            runtime.reset();
        } else if (gamepad1.dpad_right && RightClaw.getPosition() < 0.5 && (runtime.milliseconds()) > buttondelaytime) {
            RightClaw.setPosition(0.5);
            autodeposit = false;
            runtime.reset();
        }

        if (gamepad1.back) {        //Grip Pixels
            RightClaw.setPosition(0.5);
            LeftClaw.setPosition(0.5);
            autodeposit = false;
        } else if (gamepad1.start) {        //Release Pixels
            RightClaw.setPosition(0);
            LeftClaw.setPosition(1);
            autodeposit = false;
        }

        /**Intake servo*/

        if (gamepad1.y && IntakeServopos < 0.6 && (runtime.milliseconds()) > buttondelaytime) {
            IntakeServopos = IntakeServopos + 0.05;
            runtime.reset();
        }
//        else if(gamepad1.y && IntakeServopos < 0.8 && (runtime.milliseconds()) > buttondelaytime) {
//            IntakeServopos = IntakeServopos + 0.1;
//            runtime.reset();
//        }
        else if (gamepad1.y && (runtime.milliseconds()) > buttondelaytime) {
            IntakeServopos = 0.3;
            runtime.reset();
        }

        Intake_Servo.setPosition(IntakeServopos);

        /**Deposit Rotate*/

        if (gamepad1.b && DepositRotate.getPosition() > 0 && (runtime.milliseconds()) > buttondelaytime) {
            DepositRotate.setPosition(0);
            runtime.reset();
        } else if (gamepad1.b && (runtime.milliseconds()) > buttondelaytime && delivery.Pivot.getCurrentPosition() < -800) {
            DepositRotate.setPosition(0.5);
            runtime.reset();
        }

        /**Call all PID's and telemetry code*/

        Top_Pivot_Position_With_Feedforward();

        TelemetryMap();

        /** pivot flip under slides**/
//
//        if (gamepad1.left_bumper && deliverySlides.Left_Slide.getCurrentPosition() < 15) {
//            deliverySlides.DeliverySlides(400, 1);
//        } else if (gamepad1.left_bumper && deliverySlides.Left_Slide.getCurrentPosition() > 380 && delivery.Pivot.getCurrentPosition() < 100) {
//            deliverySlides.DeliverySlides(0, -1);
//        }
//
//        if (deliverySlides.Left_Slide.getVelocity() > 200 && deliverySlides.Left_Slide.getCurrentPosition() > 200) {
//                Pivot_Target = 400;
//        }

    }

    @Override
    public void init() {

        drive.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.init(hardwareMap);

        left_Pixel = hardwareMap.get(DistanceSensor.class, "left_pixel_sensor");

        right_Pixel = hardwareMap.get(ColorSensor.class, "right_pixel_sensor");

        LeftClaw = hardwareMap.get(Servo.class, "LeftPixel");

        RightClaw = hardwareMap.get(Servo.class, "RightPixel");

        Intake_Servo = hardwareMap.get(Servo.class, "intake_servo");

        DepositRotate = hardwareMap.get(Servo.class, "DepositRotate");

        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);

        currentGamepad1 = new Gamepad();

        previousGamepad1 = new Gamepad();

        runtime.reset();

        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);

        for (LynxModule hub : allHubs) {
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        Intake_Servo.setPosition(0.7);
        DepositRotate.setPosition(0);
        LeftClaw.setPosition(0.5);
        RightClaw.setPosition(0.5);
    }

    public void Top_Pivot_Position(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.3;

        delivery.Pivot.setPower(Top_Pivot_PID);

    }

    public void Top_Pivot_Position_With_Feedforward(){

        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);

        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();

        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.4;

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
        telemetry.addData("Left Pixel", LeftClaw.getPosition());
        telemetry.addData("Right Pixel", RightClaw.getPosition());
        telemetry.addData("Deposit Rotate", DepositRotate.getPosition());
        telemetry.addData("Right_Pixel_Sensor Green", right_Pixel.green());
        telemetry.addData("pivot pos", delivery.Pivot.getCurrentPosition());
        telemetry.addData("Intake servo pos", Intake_Servo.getPosition());
        telemetry.addData("pivot target", Pivot_Target);
        telemetry.update();
    }

}
