package org.firstinspires.ftc.teamcode.hardware.SubSystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Collection {

    public DcMotorEx Intake;

    public Servo IntakeHeight;

    public static double pivot_p = 0.004, pivot_i = 0, pivot_d = 0.0001;

    HardwareMap hmap;

    public void init(HardwareMap hardwareMap){
        
        hmap = hardwareMap;

        Intake = hardwareMap.get(DcMotorEx.class, "Intake");

        IntakeHeight = hardwareMap.get(Servo.class, "IntakeServo");

        IntakeHeight.setDirection(Servo.Direction.FORWARD);

        Intake.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        Intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }
}
