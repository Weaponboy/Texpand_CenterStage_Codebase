//package org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Blue_Auto.Right;
//
//import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Constants.propPos;
//
//import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
//import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
//
//import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
//import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.Vector2D;
//import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
//import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.blueRightBuilder;
//import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.redRightBuilder;
//import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Collection;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Delivery_Slides;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Drivetrain;
//import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;
//import org.firstinspires.ftc.teamcode.hardware.Method_Interfaces.Auto_Methods;
//import org.firstinspires.ftc.vision.VisionPortal;
//
//@Autonomous
///**start red right*/
//public class Blue_Right_Preload extends LinearOpMode implements Auto_Methods {
//
//    public WebcamName frontCam;
//
//    public VisionPortal portal;
//
//    org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount propDetectionByAmount = new propDetectionByAmount(telemetry, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.Side.left, org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.propDetectionByAmount.color.red);
//
//    /**hardware objects*/
//    Odometry odometry = new Odometry(210, 337, 90);
//
//    Drivetrain drive = new Drivetrain();
//
//    /**pathing objects*/
//    blueRightBuilder preloadPurple = new blueRightBuilder();
//
//    blueRightBuilder preloadYellow = new blueRightBuilder();
//
//    blueRightBuilder collect = new blueRightBuilder();
//
//    blueRightBuilder deliver = new blueRightBuilder();
//
//    mecanumFollower follower = new mecanumFollower();
//
//    Delivery delivery = new Delivery();
//
//    Delivery_Slides deliverySlides = new Delivery_Slides();
//
//    Collection collection = new Collection();
//
//    @Override
//    public void runOpMode() throws InterruptedException {
//
//        initialize();
//
//        waitForStart();
//
//
//        if (propPos == 1){
//
//            portal.close();
//
//            preloadPurple.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.purple);
//
//            preloadYellow.buildPath(blueRightBuilder.Position.left, blueRightBuilder.Section.preload, blueRightBuilder.pixelColor.yellow);
//
//            follower.setPath(preloadPurple.followablePath, preloadPurple.pathingVelocity);
//
//            //change target heading after dropping the purple pixel
//            Vector2D point;
//            follower.followPath(45, odometry, drive, point = new Vector2D(229, 260), 180);
//
//            odometry.update();
//
////            dropYellowPixel();
//
//            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);
//
//            follower.followPath(180, odometry, drive);
//
//        } else if (propPos == 2) {
//
//            portal.close();
//
//            firstPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.preload);
//
//            secondPath.buildPath(redRightBuilder.Position.center, redRightBuilder.Section.collect);
//
//            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);
//
//            //change target heading after dropping the purple pixel
//            Vector2D point;
//            follower.followPath(90, odometry, drive, point = new Vector2D(210, 300), 180);
//
//            odometry.update();
//
////            dropYellowPixel();
//
//            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);
//
//            follower.followPath(180, odometry, drive);
//
//        } else if (propPos == 3) {
//
//            portal.close();
//
//            firstPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.preload);
//
//            secondPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.collect);
//
//            thridPath.buildPath(redRightBuilder.Position.right, redRightBuilder.Section.deliver);
//
//            follower.setPath(firstPath.followablePath, firstPath.pathingVelocity);
//
//            //change target heading after dropping the purple pixel
//            Vector2D point;
//            follower.followPath(130, odometry, drive, point = new Vector2D(250, 314), 180);
//
//            odometry.update();
//
//            dropYellowPixel();
//
//            follower.setPath(secondPath.followablePath, secondPath.pathingVelocity);
//
//            follower.followPath(180, odometry, drive);
//
//            follower.setPath(thridPath.followablePath, thridPath.pathingVelocity);
//
//            follower.followPath(180, odometry, drive);
//
//        }
//
//
//    }
//
//    private void initialize(){
//
//        //init hardware
//        odometry.init(hardwareMap);
//
//        drive.init(hardwareMap);
//
//        collection.init(hardwareMap);
//
//        delivery.init(hardwareMap);
//
//        deliverySlides.init(hardwareMap);
//
//        odometry.update();
//
//        frontCam = hardwareMap.get(WebcamName.class, "frontcam");
//
//        portal = VisionPortal.easyCreateWithDefaults(frontCam, propDetectionByAmount);
//
//    }
//
//}
