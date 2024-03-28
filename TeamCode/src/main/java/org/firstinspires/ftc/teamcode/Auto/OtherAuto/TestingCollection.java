package org.firstinspires.ftc.teamcode.Auto.OtherAuto;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware.Collection;
import org.firstinspires.ftc.teamcode.hardware.Delivery;
import org.firstinspires.ftc.teamcode.hardware.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.Odometry;
import org.firstinspires.ftc.teamcode.hardware.Sensors;

@TeleOp
public class TestingCollection extends LinearOpMode {

    GenMethods preloadPaths = new GenMethods();

    Odometry odometry = new Odometry(312, 143, 180);

    Drivetrain drive = new Drivetrain();

    Delivery delivery = new Delivery();

    Delivery_Slides deliverySlides = new Delivery_Slides();

    Collection collection = new Collection();

    Sensors sensors = new Sensors();

    mecanumFollower follower = new mecanumFollower();

    Vector2D CS1F = new Vector2D(getRealCoords(312), getRealCoords(143));
    Vector2D CC1F = new Vector2D(getRealCoords(292), getRealCoords(154));
    Vector2D CE1F = new Vector2D(getRealCoords(240), getRealCoords(150));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(42), getRealCoords(150));

    @Override
    public void runOpMode() throws InterruptedException {

        drive.init(hardwareMap);

        odometry.init(hardwareMap);

        delivery.init(hardwareMap);

        deliverySlides.init(hardwareMap);

        sensors.init(hardwareMap);

        collection.init(hardwareMap);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        preloadPaths.threePoints(CS1F, CC1F, CE1F);
        preloadPaths.twoPoints(CS2F, CE2F, true);

        follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);

        waitForStart();

//        collection.setIntakeHeight(Collection.intakeHeightState.hangStowed);
//        collection.updateIntakeHeight();

        while (Math.abs(42 - odometry.X) > 1.4 && Math.abs(150 - odometry.Y) > 1.4){
            follower.followPathAuto(180, odometry, drive);
        }

//        drive.setAllPower(0);
//
//        collectPixels();

        while (opModeIsActive()){
            telemetry.addData("X", odometry.X);
            telemetry.addData("Y", odometry.Y);
            telemetry.update();
        }

    }

    public void collectPixels(){

        boolean gotTwo;

        delivery.ArmExtension.setPosition(1);

        delivery.setGripperState(Delivery.GripperState.open);
        delivery.updateGrippers();

        collection.setState(Collection.intakePowerState.on);
        collection.updateIntakeState();

        sleep(500);

        collection.setIntakeHeight(Collection.intakeHeightState.fifthPixel);
        collection.updateIntakeHeight();

        gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

        if (gotTwo){
            delivery.setGripperState(Delivery.GripperState.closed);
            delivery.updateGrippers();
        }else {
            sleep(500);

            collection.setIntakeHeight(Collection.intakeHeightState.forthPixel);
            collection.updateIntakeHeight();

            gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

            if (gotTwo){
                delivery.setGripperState(Delivery.GripperState.closed);
                delivery.updateGrippers();
            }else {
                sleep(500);

                gotTwo = !sensors.RightClawSensor.isPressed() && !sensors.LeftClawSensor.isPressed();

                if (gotTwo){
                    delivery.setGripperState(Delivery.GripperState.closed);
                    delivery.updateGrippers();
                }else {
                    sleep(500);

                    delivery.setGripperState(Delivery.GripperState.closed);
                    delivery.updateGrippers();
                }
            }

        }


        sleep(200);

        collection.setState(Collection.intakePowerState.reversedHalf);
        collection.updateIntakeState();

        collection.setIntakeHeight(Collection.intakeHeightState.startingBox);
        collection.updateIntakeHeight();

        sleep(200);

        collection.setState(Collection.intakePowerState.off);
        collection.updateIntakeState();
    }


}
