package org.firstinspires.ftc.teamcode.hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

public class planeLauncher {

    HardwareMap hardwareMap;

    public double getTriggerPosition() {
        return Trigger.getPosition();
    }

    public void setTrigger(double trigger) {
        Trigger.setPosition(trigger);
    }

    ServoImplEx Trigger;

    public void init(HardwareMap hmap){

        hardwareMap = hmap;

        Trigger = hardwareMap.get(ServoImplEx.class, "trigger");

        Trigger.setPwmRange(new PwmControl.PwmRange(600, 2400));

//        Trigger.setPosition(1);

    }


}
