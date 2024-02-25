package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.ScrimmageTeleops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware._.Odometry;
import org.firstinspires.ftc.teamcode.hardware._.Sensors;

import java.util.Objects;

@TeleOp
public class TestingArmPositions extends OpMode {

    ServoImplEx secondPivotLeft;
    ServoImplEx secondPivotRight;

    ServoImplEx RightClaw;
    ServoImplEx LeftClaw;

    ServoImplEx RotateClaw;

    ServoImplEx mainPivotLeft;
    ServoImplEx mainPivotRight;

    ServoImplEx secondRotate;

    ServoImplEx DiffPivot;

    ElapsedTime pivotMoveTimeDelivery = new ElapsedTime();
    ElapsedTime pivotMoveTimeAuto = new ElapsedTime();
    ElapsedTime pivotMoveTimeCollection = new ElapsedTime();

    Sensors sensors = new Sensors();

    double timeToWaitDelivery;
    double timeToWaitCollection;

    double timePerDegree = 7;

    double collectTopPivotPos = 0.23;
    double intermediateTopPivot = 0.3;
    double deliveryTopPivot = 0.7;
    double deliveryTopPivotNew = 1;
    double safeTopPivot = 0.3;

    double deliveryTopPivotAuto = 0.73;
    double deliverySecondPivotAuto = 0.2;
    double distancecalc;
    double avoidIntakeSecondPivot = 0.8;
    double collectSecondPivot = 0.85;
    double deliverySecondPivot = -0.16;
    double lowdeliveryTopPivot = 1;

    double clawOpenDeliver = 0.5;
    double clawOpen = 0.4;
    double clawClosed = 1;

    double rotateCollect = 0.55;
    double rotateDeliver = 0.55;
    double rotateRight = 1;

    double secondRotateMiddle = 0.5;
    double secondRotateMiddleCollect = 0.5;

    double targetRotatePos;
    static final double adjustFactor = 1.1;
    boolean closeToCollection;

    public void setMainPivotOffSet(double mainPivotOffSet) {
        this.mainPivotOffSet = mainPivotOffSet;
    }
    boolean firtloop = true;
    double mainPivotOffSet = 0;
    double targetMainPivot = 0;
    static final double servoPosPerTick = 0.00004100;
    static final double mainToSecondConst = 0.5/0.3;
    static final double mainservopospermm = 0.0126;
    static final double mindistancemm = 8;
    static final double maxdistancemm = 20;
    static final double distanceToPivot = 0.1495;

    boolean DeliveryMovingAuto = false;
    boolean DeliveryMoving = false;
    boolean CollectionMoving = false;
    boolean intermediateMoving = false;

    double deliveryIncrement = 0.02;

    HardwareMap hmap;

    @Override
    public void init() {
        init(hardwareMap);
    }

    @Override
    public void loop() {

        if (gamepad1.dpad_up){
            setMainPivot(getMainPivotPosition()+0.01);
        } else if (gamepad1.dpad_down) {
            setMainPivot(getMainPivotPosition()-0.01);
        }

        if (gamepad1.left_bumper){
            setSecondPivot(getSecondPivotPosition()+0.01);
        } else if (gamepad1.right_bumper) {
            setSecondPivot(getSecondPivotPosition()-0.01);
        }

        if (gamepad1.dpad_left){
            DiffPivot.setPosition(DiffPivot.getPosition()+0.01);
        } else if (gamepad1.dpad_right) {
            DiffPivot.setPosition(DiffPivot.getPosition()-0.01);
        }

        if (gamepad1.left_trigger > 0){
            secondRotate.setPosition(secondRotate.getPosition()+0.01);
        } else if (gamepad1.right_trigger > 0) {
            secondRotate.setPosition(secondRotate.getPosition()-0.01);
        }

        if (gamepad1.x){
            RotateClaw.setPosition(RotateClaw.getPosition()+0.01);
        } else if (gamepad1.y) {
            RotateClaw.setPosition(RotateClaw.getPosition()-0.01);
        }

    }

    public void init(HardwareMap hardwareMap){
        hmap = hardwareMap;

        pivotMoveTimeDelivery.reset();
        pivotMoveTimeCollection.reset();

        sensors.init(hardwareMap);

        /**servos init*/

        secondPivotLeft = hardwareMap.get(ServoImplEx.class, "RightPivot");
        secondPivotRight = hardwareMap.get(ServoImplEx.class, "LeftPivot");

        secondPivotRight.setDirection(Servo.Direction.REVERSE);

        secondPivotLeft.setPwmRange(new PwmControl.PwmRange(600, 2500));
        secondPivotRight.setPwmRange(new PwmControl.PwmRange(600, 2500));

        RightClaw = hardwareMap.get(ServoImplEx.class, "RightClaw");
        LeftClaw = hardwareMap.get(ServoImplEx.class, "LeftClaw");

        RightClaw.setDirection(Servo.Direction.FORWARD);
        LeftClaw.setDirection(Servo.Direction.REVERSE);

        RightClaw.setPwmRange(new PwmControl.PwmRange(600, 2500));
        LeftClaw.setPwmRange(new PwmControl.PwmRange(600, 2500));

        mainPivotLeft = hardwareMap.get(ServoImplEx.class, "leftmain");
        mainPivotRight = hardwareMap.get(ServoImplEx.class, "rightmain");

        mainPivotLeft.setDirection(Servo.Direction.REVERSE);

        mainPivotLeft.setPwmRange(new PwmControl.PwmRange(600, 2500));
        mainPivotRight.setPwmRange(new PwmControl.PwmRange(600, 2500));

        RotateClaw = hardwareMap.get(ServoImplEx.class, "ClawRotate");

        RotateClaw.setDirection(Servo.Direction.FORWARD);

        RotateClaw.setPwmRange(new PwmControl.PwmRange(700, 2300));

        secondRotate = hardwareMap.get(ServoImplEx.class, "secondRotate");

        secondRotate.setPwmRange(new PwmControl.PwmRange(600, 2500));

        DiffPivot = hardwareMap.get(ServoImplEx.class, "DiffPivot");

        DiffPivot.setPosition(0.5);

        setMainPivot(collectTopPivotPos);

        secondRotate.setPosition(secondRotateMiddleCollect);

        setSecondPivot(collectSecondPivot);

        RotateClaw.setPosition(rotateCollect);

        RightClaw.setPosition(clawOpen);
        LeftClaw.setPosition(clawOpen);

    }

    public void setRotateClaw(double position){
        RotateClaw.setPosition(position);
    }

    public double getSecondPivotPosition(){
        return secondPivotLeft.getPosition() + secondPivotRight.getPosition()/2;
    }

    public double getMainPivotPosition(){
        return (mainPivotLeft.getPosition() + mainPivotRight.getPosition()) / 2;
    }

    private void setMainPivot(double position){
        mainPivotLeft.setPosition(position);
        mainPivotRight.setPosition(position);
    }

    public double getTopPivotPosition(){
        return secondPivotLeft.getPosition();
    }

    private void setClaws(double position){
        LeftClaw.setPosition(position);
        RightClaw.setPosition(position);
    }

    private void setSecondPivot(double position){
        secondPivotLeft.setPosition(position);
        secondPivotRight.setPosition(position);
    }

}
