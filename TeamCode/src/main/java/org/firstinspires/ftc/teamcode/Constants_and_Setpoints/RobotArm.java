package org.firstinspires.ftc.teamcode.Constants_and_Setpoints;

public class RobotArm {

    public double getMainPivot() {
        return mainPivot;
    }

    public double getArmRotate() {
        return armRotate;
    }

    public double getSecondRotate() {
        return secondRotate;
    }

    public double getSecondPivot() {
        return secondPivot;
    }

    public double getClawRotate() {
        return ClawRotate;
    }

    public double getArmExtension() {
        return ArmExtension;
    }

    public int getSlides() {
        return slides;
    }

    double mainPivot;
    double armRotate;
    double secondRotate;
    double secondPivot;
    double ClawRotate;
    double ArmExtension;
    int slides;

    public RobotArm(double mainPivot, double armRotate, double secondRotate, double secondPivot, double ClawRotate, double ArmExtension, int slides) {
        this.mainPivot = mainPivot;
        this.armRotate = armRotate;
        this.secondRotate = secondRotate;
        this.secondPivot = secondPivot;
        this.ClawRotate = ClawRotate;
        this.slides = slides;
        this.ArmExtension = ArmExtension;
    }
}








