package org.firstinspires.ftc.teamcode.Testing.Test_Autos;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.UsefulMethods.getRealCoords;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Auto.Comp_Autos.Methods.CycleMethods;
import org.firstinspires.ftc.teamcode.Odometry.ObjectAvoidance.old.Vector2D;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.Follower.mecanumFollower;
import org.firstinspires.ftc.teamcode.Odometry.Pathing.PathGeneration.pathBuilderSubClasses.GenMethods;
import org.firstinspires.ftc.teamcode.hardware._.Collection;
import org.firstinspires.ftc.teamcode.hardware._.Delivery;
import org.firstinspires.ftc.teamcode.hardware._.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;

@Autonomous(name = "Collection Testing", group = "Newest auto's")
public class CollectionAutoTesting extends LinearOpMode implements CycleMethods {

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
    Vector2D DPC1F = new Vector2D(getRealCoords(241), getRealCoords(45));
    Vector2D DPCT1F = new Vector2D(getRealCoords(188), getRealCoords(83));
    Vector2D DPE1F = new Vector2D(getRealCoords(305), getRealCoords(82.5));

    //second pos
    Vector2D DPS1S = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1S = new Vector2D(getRealCoords(220), getRealCoords(76));
    Vector2D DPCT1S = new Vector2D(getRealCoords(170), getRealCoords(65));
    Vector2D DPE1S = new Vector2D(getRealCoords(295), getRealCoords(95));

    Vector2D DPS1T = new Vector2D(getRealCoords(210), getRealCoords(23));
    Vector2D DPC1T = new Vector2D(getRealCoords(223), getRealCoords(79));
    Vector2D DPCT1T = new Vector2D(getRealCoords(164), getRealCoords(110));
    Vector2D DPE1T = new Vector2D(getRealCoords(305), getRealCoords(105));

    Vector2D DS1F = new Vector2D(getRealCoords(300), getRealCoords(121));

    Vector2D CS1F = new Vector2D(getRealCoords(300), getRealCoords(90));
    Vector2D CC1F = new Vector2D(getRealCoords(263), getRealCoords(162));
    Vector2D CE1F = new Vector2D(getRealCoords(179), getRealCoords(152));

    Vector2D CS2F = CE1F;
    Vector2D CE2F = new Vector2D(getRealCoords(65), getRealCoords(150));

    Vector2D lastPoint = new Vector2D(getRealCoords(41), getRealCoords(150));


    GenMethods preloadPaths = new GenMethods();
    GenMethods collect = new GenMethods();
    GenMethods deliver = new GenMethods();
    GenMethods deliverLast = new GenMethods();

    /**hardware objects*/
    Odometry odometry = new Odometry(120, 150, 180);

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

    Phase phase = Phase.preload;

    Build build = Build.notBuilt;

    Auto auto = Auto.preload;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();

        preloadPaths.twoPoints(new Vector2D(120, 150), CE2F, true);

        deliverLast.twoPoints(CE2F, lastPoint, true);

        while (!(phase == Phase.finished)){
            switch (phase){
                case preload:

                    odometry.update();

                    if (build == Build.notBuilt){
                        follower.setPath(preloadPaths.followablePath, preloadPaths.pathingVelocity);
                        pathing = true;
                        build = Build.built;
                        targetHeading = 180;
                        delivery.setGripperState(Delivery.GripperState.open);
                        delivery.updateGrippers();
                        collection.setIntakeHeight(Collection.intakeHeightState.stowed);
                        collection.updateIntakeHeight();
                        collection.setState(Collection.intakePowerState.onHalf);
                        collection.updateIntakeState();
                    }

                    if (pathing){

                        pathing = follower.followPathAuto(targetHeading, odometry, drive);

                    } else if (Math.abs(65 - odometry.X) < 6 && Math.abs(150 - odometry.Y) < 6) {

                        drive.RF.setPower(0);
                        drive.RB.setPower(0);
                        drive.LF.setPower(0);
                        drive.LB.setPower(0);

                        phase = Phase.first2;

                        build = Build.notBuilt;
                    }

                    break;
                case first2:

                    odometry.update();

                    if (build == Build.notBuilt){
                        follower.setPath(deliverLast.followablePath, deliverLast.pathingVelocity);
                        pathing = true;
                        build = Build.built;
                        targetHeading = 180;
                    }

                    if (pathing){

                        pathing = follower.followPathAuto(targetHeading, odometry, drive);

                    } else if (Math.abs(41 - odometry.X) < 5 && Math.abs(150 - odometry.Y) < 5 && !pathing) {

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

                        phase = Phase.finished;

                    }

                    break;
                default:
            }
        }

    }

    public void initialize(){
        //init hardware
        odometry.init(hardwareMap);

        drive.init(hardwareMap);

        init(hardwareMap);

        sensors.initAprilTag(telemetry, false);
    }

}
