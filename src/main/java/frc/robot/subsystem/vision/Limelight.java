package frc.robot.subsystem.vision;

import frc.lib.limelight.LimelightHelpers;

public class Limelight {
    private String limelightName;
    public Limelight(String limelightName){
        this.limelightName = limelightName;
    }

    public double getTx(){
        return LimelightHelpers.getTX(limelightName);
    }

    public double getTY(){
        return LimelightHelpers.getTY(limelightName);
    }
}
