package frc.lib.control;

public class PIDFeedBack {
    public double kP;
    public double kI;
    public double kD;

    public PIDFeedBack(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

}
