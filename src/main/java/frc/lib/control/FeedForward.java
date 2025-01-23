package frc.lib.control;

public class FeedForward {
    public double kS;
    public double kV;
    public double kA;

    public FeedForward(double kS, double kV, double kA) {
        this.kS = kS;
        this.kV = kV;
        this.kA = kA;
    }

    public double calculate(double velocity, double acceleration) {
        return kS + kV * velocity + kA * acceleration;
    }
}
