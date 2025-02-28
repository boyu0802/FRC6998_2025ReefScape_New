package frc.robot.subsystem.vision;

import java.io.IOException;
import java.util.concurrent.atomic.AtomicReference;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;

import edu.wpi.first.apriltag.AprilTagFieldLayout.OriginPosition;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;

public class PhotonRunnable implements Runnable {
    private final PhotonPoseEstimator photonPoseEstimator;
    private final PhotonCamera photonCamera;
    private final AtomicReference<EstimatedRobotPose> atomicEstimatedRobotPose = new AtomicReference<EstimatedRobotPose>();

    public PhotonRunnable(PhotonCamera photonCamera, Transform3d robotToCamera) {
        this.photonCamera = photonCamera;
        PhotonPoseEstimator photonPoseEstimator = null;

        try {
            var layout = AprilTagFields.k2025ReefscapeWelded.loadAprilTagLayoutField();
            // PV estimates will always be blue, they'll get flipped by robot thread
            layout.setOrigin(OriginPosition.kBlueAllianceWallRightSide);
            if (photonCamera != null) {
            photonPoseEstimator = new PhotonPoseEstimator(
                layout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,robotToCamera);
            }
        }
        catch (IllegalArgumentException e) { // Replace with actual exception type
                DriverStation.reportError("Failed to load reef field layout", e.getStackTrace());
            }
        this.photonPoseEstimator = photonPoseEstimator;
    }



    @Override
    public void run() {
        if (photonPoseEstimator != null && photonCamera != null) {
            var photonResults = photonCamera.getLatestResult();
            if (photonResults.hasTargets()
                && (photonResults.targets.size() > 1
                    || photonResults.targets.get(0).getPoseAmbiguity() < 0.2)) {
              photonPoseEstimator.update(photonResults).ifPresent(estimatedRobotPose -> {
                var estimatedPose = estimatedRobotPose.estimatedPose;
                // Make sure the measurement is on the field
                if (estimatedPose.getX() > 0.0 && estimatedPose.getX() <= Units.inchesToMeters(651.25)
                    && estimatedPose.getY() > 0.0 && estimatedPose.getY() <= Units.inchesToMeters(315.5)) {
                  atomicEstimatedRobotPose.set(estimatedRobotPose);
                }
              });
            }
        }
    }

    public EstimatedRobotPose grabLatestEstimatedPose() {
        return atomicEstimatedRobotPose.getAndSet(null);
    }
    
}
