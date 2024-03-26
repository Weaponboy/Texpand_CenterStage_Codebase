package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class planeLauncher {

    HardwareMap hardwareMap;

    public double getTriggerPosition() {
        return Trigger.getPosition();
    }

    public void setTrigger(double trigger) {
        Trigger.setPosition(trigger);
    }

    Servo Trigger;

    public void init(HardwareMap hmap){

        hardwareMap = hmap;

        Trigger = hardwareMap.get(Servo.class, "trigger");

        Trigger.setPosition(0.65);

    }


}
