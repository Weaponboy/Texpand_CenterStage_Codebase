package org.firstinspires.ftc.teamcode.Teleop.CompitionTeleops.ScrimmageTeleop;

import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.deliverySlides;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.drive;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Hardware_objects.sensors;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.portal;
import static org.firstinspires.ftc.teamcode.Constants_and_Setpoints.Non_Hardware_Objects.propDetecterRed;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.VisionTesting.VisionPortalProcessers.PropDetecterByHeight;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Delivery_Slides;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Drivetrain;
import org.firstinspires.ftc.teamcode.hardware.SubSystems.Sensors;
import org.firstinspires.ftc.vision.VisionPortal;


public class Scrimmage_Red extends OpMode {

    @Override
    public void init() {
        sensors = new Sensors();
        drive = new Drivetrain();
        deliverySlides = new Delivery_Slides();

        drive.init(hardwareMap);
        deliverySlides.init(hardwareMap);
        sensors.init(hardwareMap);

        propDetecterRed = new PropDetecterByHeight(PropDetecterByHeight.color.red);
        portal = VisionPortal.easyCreateWithDefaults(sensors.frontCam, propDetecterRed);
    }

    @Override
    public void loop() {
    }

}
