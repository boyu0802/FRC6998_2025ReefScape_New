package frc.robot.subsystem.vision;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.PolynomialRegression;
import frc.lib.util.TimestampedVisionUpdate;

import static frc.robot.Constants.PhotonAprilTagConstants.*;


public class PhotonAprilTagVision extends SubsystemBase {

    private final PhotonCamera[] reefCameras = new PhotonCamera[2];
    private final PhotonPoseEstimator[] reefPoseEstimator = new PhotonPoseEstimator[2];
    //private final PhotonCamera photonCamera = new PhotonCamera("PhotonCamera");
    private BiConsumer<Pose2d, Double> visionFieldPoseEstimateConsumer;

    private Supplier<Pose2d> poseSupplier;  

    private List<TimestampedVisionUpdate> visionUpdates;
    private List<TimestampedVisionUpdate> visionUpdatesAuto;

    private double stdDevScalarAuto = 0.5940;
    private double thetaStdDevCoefficientAuto = 0.1;

    private double stdDevScalarShooting = 0.2;
    private double thetaStdDevCoefficientShooting = 0.075;

    

    



    public PhotonAprilTagVision() {
        reefCameras[0] = new PhotonCamera("ReefLeft");
        reefCameras[1] = new PhotonCamera("ReefRight");

        reefPoseEstimator[0] = new PhotonPoseEstimator(TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, REEF_CAMERA_OFFSET[0]);
        reefPoseEstimator[1] = new PhotonPoseEstimator(TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, REEF_CAMERA_OFFSET[1]);

        reefPoseEstimator[0].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        reefPoseEstimator[1].setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

    }

    public Optional<Pose3d> photonGetFieldtoRobot(PhotonPoseEstimator poseEstimator, PhotonPipelineResult result, double timestamp) {
        if (result.hasTargets()) {
            return Optional.of(poseEstimator.getReferencePose());
        } else {
            return Optional.empty();
        }

    } 

    
    
    @Override
    public void periodic() {

        // This method will be called once per scheduler run

        Pose2d currentPose = poseSupplier.get();
        visionUpdates = new ArrayList<>();
        visionUpdatesAuto = new ArrayList<>();
        

        for (PhotonCamera camera : reefCameras) {
            
            Pose3d cameraPose;
            Pose2d robotPose;
            List<Pose2d> tagPose2ds = new ArrayList<>();
            
            
            PhotonPipelineResult unprocessedResult = camera.getLatestResult();

            double singleTagAdustment = 1.0;

            double timestamp = unprocessedResult.getTimestampSeconds();



            
        }

    }

    


}
