package org.firstinspires.ftc.teamcode.Odometry.Pathing.Testing;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.hardware.Base_SubSystems.Odometry;

public class ArcOdometrytesting extends LinearOpMode {

    Odometry lineOdo = new Odometry();

    Odometry arcOdo = new Odometry();

    @Override
    public void runOpMode() throws InterruptedException {

        arcOdo.init(hardwareMap);

        lineOdo.init(hardwareMap);

        waitForStart();

        while (opModeIsActive()){
            arcOdo.updateArc();
            lineOdo.update();

            telemetry.addData("arcOdo X", arcOdo.X);
            telemetry.addData("arcOdo Y", arcOdo.Y);
            telemetry.addData("arcOdo Heading", arcOdo.heading);
            telemetry.addLine();
            telemetry.addData("lineOdo X", lineOdo.X);
            telemetry.addData("lineOdo Y", lineOdo.Y);
            telemetry.addData("lineOdo Heading", lineOdo.heading);
            telemetry.update();

        }

    }

}
