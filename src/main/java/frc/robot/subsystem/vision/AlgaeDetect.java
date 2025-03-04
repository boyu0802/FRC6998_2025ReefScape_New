package frc.robot.subsystem.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import lombok.Getter;

public class AlgaeDetect extends SubsystemBase {

    private final PhotonCamera algaeCamera = new PhotonCamera("Algae");

    
    @Getter boolean targetVisible = false;

    @Getter double targetYaw = 0.0;
    @Getter double targetRange = 0.0;

    @Getter Pose2d distancePose = new Pose2d();


    public AlgaeDetect() {
        algaeCamera.setLED(VisionLEDMode.kOn);

        
        
    }

    @Override
    public void periodic() {
        var result = algaeCamera.getLatestResult();
        if (result.hasTargets()) {
            targetYaw = result.getBestTarget().getYaw();
            targetVisible = true;
            
        } else {
            targetVisible = false;
        }
    }

    
    
}
