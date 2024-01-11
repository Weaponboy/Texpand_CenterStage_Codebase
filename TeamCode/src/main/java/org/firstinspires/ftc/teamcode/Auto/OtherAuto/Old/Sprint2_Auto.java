//package org.firstinspires.ftc.teamcode.Auto;
//
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot_d;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot_i;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.pivot_p;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.delivery;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.deliverySlides;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.drive;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.sensors;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Setpoints.Pivot_Target;
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getDriveToBackboardControlPoint;
//
//import com.arcrobotics.ftclib.controller.PIDFController;
//import com.qualcomm.hardware.lynx.LynxModule;
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.Enums.TargetPoint;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Sensors;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//import java.util.List;
//
//@Autonomous
//public class Sprint2_Auto extends LinearOpMode {
//
//    double Xdist;
//
//    double Ydist;
//
//    Odometry odometry = new Odometry(93, 23, 270);
//
//    public WebcamName frontCam;
//
//    public VisionPortal portal;
//
//    Servo RightClaw;
//
//    Servo LeftClaw;
//
//    Servo DepositServoRotate;
//
//    Servo clawRight;
//
//    Servo clawLeft;
//
//    Servo DepositPivot;
//
//    PIDFController Slide_Power;
//
//    double pivotPower;
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initialization();
//
//        while (opModeInInit()){
//            odometry.update();
//            propPos = 1;
//            telemetry.addData("Position", propPos);
////            telemetry.addData("1", propDetecterRed.position1);
////            telemetry.addData("2", propDetecterRed.position2);
////            telemetry.addData("3", propDetecterRed.position3);
//            telemetry.update();
//        }
//
//        propPos = 1;
//
//        waitForStart();
//
//        if (propPos == 1){
//
//            odometry.Odo_Drive(95, 85, 180);
//
//            odometry.Odo_Drive(80, 85, 180);
//
//            odometry.Odo_Drive(100, 150, 180);
//
//            odometry.Odo_Drive(240, 150, 180);
//
//            odometry.Odo_Drive(260, 70, 180);
//
//            dropYellowPixelNew();
//
//            odometry.Odo_Drive(290, 70, 180);
//
//            sleep(1000);
//
//            clawLeft.setPosition(0.6);
//
//            sleep(500);
//
//            odometry.Odo_Drive(260, 70, 180);
//
//            dropAndRetract();
//
//            odometry.Odo_Drive(296, 70, 180);
//
//        } else if (propPos == 2) {
//
//            odometry.Odo_Drive(95, 85, 270);
//
//            odometry.Odo_Drive(95, 75, 270);
//
//            odometry.Odo_Drive(50, 75, 270);
//
//            odometry.Odo_Drive(60, 183, 180);
//
//            odometry.Odo_Drive(20, 125, 180);
//
//            odometry.Odo_Drive(240, 150, 180);
//
//            odometry.Odo_Drive(260, 90, 180);
//
//            dropYellowPixelNew();
//
//            odometry.Odo_Drive(275, 90, 180);
//
//            clawLeft.setPosition(0.6);
//
//            dropAndRetract();
//
//        } else if (propPos == 3) {
//
//            odometry.Odo_Drive(90, 65, 315);
//
//            odometry.Odo_Drive(95, 50, 270);
//
//            odometry.Odo_Drive(100, 150, 270);
//
//            odometry.Odo_Drive(240, 150, 180);
//
//            odometry.Odo_Drive(260, 110, 180);
//
//            dropYellowPixelNew();
//
//            odometry.Odo_Drive(290, 110, 180);
//
//            sleep(1000);
//
//            clawLeft.setPosition(0.6);
//
//            dropAndRetract();
//        }
//    }
//
//    public void initialization(){
//
//        drive = new Drivetrain();
//        sensors = new Sensors();
//        deliverySlides = new Delivery_Slides();
//        delivery = new Delivery();
//
//        deliverySlides.init(hardwareMap);
//        delivery.init(hardwareMap);
//        odometry.init(hardwareMap);
//        drive.init(hardwareMap);
//
////        frontCam = hardwareMap.get(WebcamName.class, "frontCam");
//
//        Slide_Power = new PIDFController(pivot_p, pivot_i, pivot_d, 0);
//
//        DepositServoRotate = hardwareMap.get(Servo.class, "DepositRotate");
//        clawLeft = hardwareMap.get(Servo.class, "clawLeft");
//        clawRight = hardwareMap.get(Servo.class, "clawRight");
//        DepositPivot = hardwareMap.get(Servo.class, "DepositPivot");
//
//        List<LynxModule> allHubs = hardwareMap.getAll(LynxModule.class);
//
//        for (LynxModule hub : allHubs) {
//            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
//        }
//
//        clawLeft.setPosition(0);
//        clawRight.setPosition(1);
//
//        DepositServoRotate.setPosition(0.5);
//        DepositPivot.setPosition(1);
//
////        propDetecterRed = new PropDetecterByHeight(PropDetecterByHeight.color.red);
////
////        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetecterRed);
//    }
//
//    public void dropYellowPixel() {
//
//        deliverySlides.DeliverySlides(150, 1);
//        while (deliverySlides        if (gamepad1.b){
//
//            if (getDriveToBackboardControlPoint(robotPos) != null){
//                path.buildPath(TargetPoint.blueBackBoard, robotPos, getDriveToBackboardControlPoint(robotPos));
//            }else {
//                path.buildPath(TargetPoint.blueBackBoard, robotPos);
//            }
//
//            targetHeading = 180;
//
//            pathing = true;
//
//            follower.setPath(path.followablePath, path.pathingVelocity);
//        }
//
//
//
////        if (gamepad1.a){
////
////            if (getDriveToCollectionControlPoint(robotPos) != null){
////                path.buildPath(TargetPoint.blueCollection, getDriveToCollectionControlPoint(robotPos), robotPos);
////            }else {
////                path.buildPath(TargetPoint.blueBackBoard, robotPos);
////            }
////
////            targetHeading = 90;
////
////            pathing = true;
////
////            follower.setPath(path.followablePath, path.pathingVelocity);
////        }.Left_Slide.getCurrentPosition() < 150) {
//
//        }
//        Pivot_Target = 100;
//
//        while (delivery.Pivot.getCurrentPosition() < Pivot_Target) {
//            Top_Pivot_Position_With_Feedforward();
//        }
//
//        RightClaw.setPosition(0);
//        sleep(200);
//
//        Pivot_Target = 0;
//
//        while (delivery.Pivot.getCurrentPosition() > Pivot_Target) {
//            Top_Pivot_Position_With_Feedforward();
//
//        }
//
//        deliverySlides.DeliverySlides(0, -1);
//
//
//    }
//
//    public void dropPurplePixel() {
//        deliverySlides.DeliverySlides(150, 1);
//
//        while (deliverySlides.Left_Slide.getCurrentPosition() < 150) {
//
//        }
//
//        Pivot_Target = 400;
//
//        while (delivery.Pivot.getCurrentPosition() < Pivot_Target) {
//            Top_Pivot_Position_With_Feedforward();
//        }
//
//        LeftClaw.setPosition(1);
//        sleep(200);
//
//        Pivot_Target = 0;
//
//        while (delivery.Pivot.getCurrentPosition() > Pivot_Target) {
//          Top_Pivot_Position_With_Feedforward();
//        }
//
//        deliverySlides.DeliverySlides(0, -1);
//
//
//    }
//
//    public void dropYellowPixelNew() {
//
//        Pivot_Target = 220;
//
//        pivotPower = 0.5;
//
//        Top_Pivot_Position(pivotPower);
//
//        while (delivery.Pivot.getCurrentPosition() < 220){
//            Top_Pivot_Position(pivotPower);
//        }
//
//        deliverySlides.DeliverySlides(-1100, -0.5);
//
//        while (deliverySlides.Left_Slide.getCurrentPosition() > -1090) {
//            Top_Pivot_Position(pivotPower);
//        }
//
//        DepositServoRotate.setPosition(1);
//
//        sleep(1000);
//
//        clawLeft.setPosition(0);
//        clawRight.setPosition(1);
//
//        Pivot_Target = -990;
//        pivotPower = 0.5;
//
//        while (delivery.Pivot.getCurrentPosition() > -850){
//            Top_Pivot_Position(pivotPower);
//        }
//
//        DepositPivot.setPosition(0.5);
//
//        DepositServoRotate.setPosition(0.5);
//
//        sleep(1000);
//
//    }
//
//    public void dropAndRetract(){
//
//        clawLeft.setPosition(0);
//        clawRight.setPosition(1);
//
//        DepositServoRotate.setPosition(1);
//
//        DepositPivot.setPosition(0.5);
//
//        try {
//            Thread.sleep(600);
//        } catch (InterruptedException e) {
//            throw new RuntimeException(e);
//        }
//
//        Pivot_Target = 140;
//
//        Top_Pivot_Position(pivotPower);
//        pivotPower = 0.5;
//
//        while (delivery.Pivot.getCurrentPosition() < 120){
//            Top_Pivot_Position(pivotPower);
//        }
//
//        DepositServoRotate.setPosition(0.5);
//        DepositPivot.setPosition(1);
//
//        sleep(1000);
//
//        deliverySlides.DeliverySlides(0, 0.5);
//
//        while (deliverySlides.Left_Slide.getCurrentPosition() < -10){
//            Top_Pivot_Position(pivotPower);
//        }
//
//        DepositServoRotate.setPosition(0.5);
//
//        Pivot_Target = 0;
//
//        Top_Pivot_Position(pivotPower);
//    }
//
//    public void Top_Pivot_Position_With_Feedforward() {
//
//        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);
//
//        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();
//
//        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * 0.6;
//
//        double pivot_f = 0.1;
//
//        double ticks_in_degrees = 2550 / 180.0;
//
//        double Pivot_FF = Math.cos(Math.toRadians(Pivot_Target / ticks_in_degrees)) * pivot_f;
//
//        double Pivot_Power = Top_Pivot_PID + Pivot_FF;
//
//        delivery.Pivot.setPower(Pivot_Power);
//
//    }
//
//    public void Top_Pivot_Position(double power){
//
//        delivery.pivot_controllers.setPIDF(pivot_p, pivot_i, pivot_d, 0);
//
//        double Pivot_Current_Position = delivery.Pivot.getCurrentPosition();
//
//        double Top_Pivot_PID = delivery.pivot_controllers.calculate(Pivot_Current_Position, Pivot_Target) * power;
//
//        delivery.Pivot.setPower(Top_Pivot_PID);
//
//    }
//
//}
