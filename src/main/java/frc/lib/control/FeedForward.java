package frc.lib.control;

import lombok.Builder;
import lombok.Data;


@Data
@Builder
public class FeedForward {
    private double kS;
    private double kV;
    private double kA;
    private double kG;

    private GravityType gravityType;

    public enum GravityType {
        kArm,
        kElevator,
    }
    
}

