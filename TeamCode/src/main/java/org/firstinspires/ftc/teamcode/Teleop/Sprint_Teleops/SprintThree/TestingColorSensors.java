package org.firstinspires.ftc.teamcode.Teleop.Sprint_Teleops.SprintThree;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;

@TeleOp
@Disabled
public class TestingColorSensors extends OpMode {

    ColorSensor rightTest;
    ColorSensor leftTest;

    @Override
    public void init() {

        rightTest = hardwareMap.get(ColorSensor.class, "right");
        leftTest = hardwareMap.get(ColorSensor.class, "left");

    }

    @Override
    public void loop() {
        telemetry.addData("right color I2C", rightTest.getI2cAddress());
        telemetry.addData("left color I2C", leftTest.getI2cAddress());
        telemetry.addData("right color blue", rightTest.blue());
        telemetry.addData("left color blue", leftTest.blue());
        telemetry.update();

    }

}
