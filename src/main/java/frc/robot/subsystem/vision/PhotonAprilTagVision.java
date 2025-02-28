package frc.robot.subsystem.vision;

import java.text.DecimalFormat;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Optional;
import java.util.function.BiConsumer;
import java.util.function.Supplier;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.util.GeomUtil;
import frc.lib.util.PolynomialRegression;
import frc.lib.util.TimestampedVisionUpdate;
import frc.lib.util.Trio;
import frc.robot.Robot;
import lombok.Data;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.Constants.PhotonAprilTagConstants.*;


public class PhotonAprilTagVision extends SubsystemBase {

    private final VisionState visionState;

    // List of cameras

    private final PhotonCamera reefLeftCamera = new PhotonCamera("ReefLeft");
    private final PhotonCamera reefRightCamera = new PhotonCamera("ReefRight");

    private final PhotonCamera[] reefCameras = {reefLeftCamera, reefRightCamera};

    private final PhotonPoseEstimator reefLefEstimator = new PhotonPoseEstimator(TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, REEF_CAMERA_OFFSET[0]);
    private final PhotonPoseEstimator reefRightEstimator = new PhotonPoseEstimator(TAG_FIELD_LAYOUT, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, REEF_CAMERA_OFFSET[1]);
    
    

    private final PhotonPoseEstimator[] reefPoseEstimator = {reefLefEstimator, reefRightEstimator};
    //private final PhotonCamera photonCamera = new PhotonCamera("PhotonCamera");
    

    private final DecimalFormat df = new DecimalFormat();

    public ArrayList<Trio<Pose3d, Pose2d, Double>> autonPoses =
            new ArrayList<Trio<Pose3d, Pose2d, Double>>();

    private boolean isAiming = false;

    private Supplier<Pose2d> poseSupplier;  

    private List<TimestampedVisionUpdate> visionUpdates;
    private List<TimestampedVisionUpdate> visionUpdatesAuto;

    private double stdDevScalarAuto = 0.5940;
    private double thetaStdDevCoefficientAuto = 0.1;
    private double singleTagAdjustment = 1.0;
    private double singleTagStdDevScalar = 100.0;

    private double stdDevScalarShooting = 0.2; // throw coral.
    private double thetaStdDevCoefficientShooting = 0.075;

    private final HashMap<PhotonCamera,Matrix<N3,N1>> cameraStdDevMap = new HashMap<>(); 

    
    public PhotonAprilTagVision(VisionState visionState) {
        this.visionState = visionState;

        reefLefEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
        reefRightEstimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);

        initializeCameraStdDevs();
        
        df.setMaximumFractionDigits(2);      
      
    }

    private void initializeCameraStdDevs() {
        for (PhotonCamera camera : reefCameras) {
            cameraStdDevMap.put(camera, kSingleTagStdDevs);
        }
    }

    // comparsion.

     public Optional<EstimatedRobotPose> getEstimatedGlobalPose(PhotonCamera photonCamera,PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = Optional.empty();
        for (var change : photonCamera.getAllUnreadResults()) {
            visionEst = photonEstimator.update(change);
            updateEstimationStdDevs(photonCamera, photonEstimator, visionEst, change.getTargets());

        }

        return visionEst;
    }

    public Optional<VisionFieldPoseEstimate> getVisionFieldPoseEstimate(PhotonCamera photonCamera,PhotonPoseEstimator photonEstimator) {
        Optional<EstimatedRobotPose> visionEst = getEstimatedGlobalPose(photonCamera,photonEstimator);
        if (visionEst.isEmpty()) {
            return Optional.empty();
        }
        else {
            return Optional.of(new VisionFieldPoseEstimate(
                visionEst.get().estimatedPose.toPose2d(), 
                visionEst.get().timestampSeconds, 
                getCameraStdDev(photonCamera)));
        
        }
        
    }

    public Optional<VisionFieldPoseEstimate> getVisionFieldPoseEstimate(PhotonCamera camera,EstimatedRobotPose visionEst) {
        return Optional.of(new VisionFieldPoseEstimate(visionEst.estimatedPose.toPose2d(), visionEst.timestampSeconds, getCameraStdDev(camera)));
    }


    // update std dev.


    private void updateEstimationStdDevs(
            PhotonCamera camera,PhotonPoseEstimator photonEstimator,Optional<EstimatedRobotPose> estimatedPose, List<PhotonTrackedTarget> targets) {

        var estStdDevs = kSingleTagStdDevs;
        int numTags = 0;
        double totalDist = 0;
        double avgDist = 0;
        double xyStdDev = 0.0;
        double thetaStdDev = 0.0;
        
        if (estimatedPose.isEmpty()) {
            // No pose input. Default to single-tag std devs
            estStdDevs = kSingleTagStdDevs;

        } else {
            // Pose present. Start running Heuristic
            

            // Precalculation - see how many tags we found, and calculate an average-distance metric
            for (var tgt : targets) {
                var tagPose = photonEstimator.getFieldTags().getTagPose(tgt.getFiducialId());
                if (tagPose.isEmpty()) continue;
                numTags++;
                avgDist +=
                        tagPose
                                .get()
                                .toPose2d()
                                .getTranslation()
                                .getDistance(estimatedPose.get().estimatedPose.toPose2d().getTranslation());
            }

            if (numTags == 0) {
                // No tags visible. Default to single-tag std devs
                estStdDevs = kSingleTagStdDevs;
            } else {
                // One or more tags visible, run the full heuristic.
                totalDist = avgDist;
                avgDist /= numTags;

                

                // Decrease std devs if multiple targets are visible
                if (numTags > 1) {
                    xyStdDev = Math.pow(avgDist, 2.0) / numTags;
                    thetaStdDev = Math.pow(avgDist, 2.0) / numTags;
                    estStdDevs = VecBuilder.fill(xyStdDev*stdDevScalarShooting, xyStdDev*stdDevScalarShooting, thetaStdDev*stdDevScalarShooting);
                }
                // Increase std devs based on (average) distance
                if (numTags == 1 && avgDist > 4){
                    estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
                }
                    
                else {
                    xyStdDev = xyStdDevModel.predict(avgDist);
                    thetaStdDev = thetaStdDevModel.predict(avgDist);
                    estStdDevs = VecBuilder.fill(xyStdDev*stdDevScalarShooting, xyStdDev*stdDevScalarShooting, thetaStdDev*stdDevScalarShooting);
                }
                
            }
        }
        updateCameraStdDev(camera, estStdDevs);
        //return estStdDevs;
    }

    public void updateCameraStdDev(PhotonCamera camera, Matrix<N3, N1> newStdDev) {
        if (camera == null || newStdDev == null) {
            throw new IllegalArgumentException("can't be null!");
        }
        cameraStdDevMap.put(camera, newStdDev);
    }

    // 批量更新所有相機的標準差矩陣
    public void updateAllCameraStdDevs(Matrix<N3, N1>[] newStdDevs) {
        if (newStdDevs.length != reefCameras.length) {
            throw new IllegalArgumentException("Illegal length of newStdDevs!");
        }
        for (int i = 0; i < reefCameras.length; i++) {
            if (newStdDevs[i] != null) {
                cameraStdDevMap.put(reefCameras[i], newStdDevs[i]);
            }
        }
    }

    // 獲取某個相機的標準差矩陣（確保不會返回 null）
    
    // 安全獲取標準差矩陣
    public Matrix<N3, N1> getCameraStdDev(PhotonCamera camera) {
        return cameraStdDevMap.getOrDefault(camera, kSingleTagStdDevs);
    }

    
        
    

    
    @Override
    public void periodic() {

        // This method will be called once per scheduler run

        // 選擇最新的 VisionPose
        Optional<EstimatedRobotPose> bestVisionEst = Optional.empty();
        double latestTimestamp = -1.0;

        for (PhotonCamera camera : reefCameras) {
            if(camera == null || reefPoseEstimator == null) {
                continue;
            }
            for (var change : camera.getAllUnreadResults()) {
                for (PhotonPoseEstimator estimator : reefPoseEstimator) {
                    Optional<EstimatedRobotPose> currentEst = estimator.update(change);
                    
                    if (currentEst.isPresent() && currentEst.get().timestampSeconds > latestTimestamp) {
                        bestVisionEst = currentEst;
                        latestTimestamp = currentEst.get().timestampSeconds;
                        getVisionFieldPoseEstimate(camera, estimator);
                        visionState.addVisionFieldPoseEstimate(getVisionFieldPoseEstimate(camera, estimator).get());
                    }

                    updateEstimationStdDevs(camera, estimator, currentEst, change.getTargets());
                }
            }
        }
    }


}
