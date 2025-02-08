package frc.robot.subsystem.vision;
import java.util.Optional;
import java.util.function.Consumer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.interpolation.TimeInterpolatableBuffer;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystem.vision.VisionFieldPoseEstimate;

public class VisionState {
    private final TimeInterpolatableBuffer<Pose2d> poseBuffer = TimeInterpolatableBuffer.createBuffer(1.0);
    private final TimeInterpolatableBuffer<Double> AngularYawVelocity = TimeInterpolatableBuffer.createDoubleBuffer(1.0);
    private final Consumer<VisionFieldPoseEstimate> visionFieldPoseEstimateConsumer;
    private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
    private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();

    public VisionState(Consumer<VisionFieldPoseEstimate> visionFieldPoseEstimateConsumer) {
        this.visionFieldPoseEstimateConsumer = visionFieldPoseEstimateConsumer;
        poseBuffer.addSample(0.0, new Pose2d());
        AngularYawVelocity.addSample(0.0, 0.0);
    }

    public void addOdometryMeasurement(double timestamp, Pose2d pose) {
        poseBuffer.addSample(timestamp, pose);
        
    }

    public void addDriveMeasurement(double timestamp, double AngularYawVelocity, ChassisSpeeds robotRelativeSpeeds, ChassisSpeeds fieldRelativeSpeeds) {
        this.AngularYawVelocity.addSample(timestamp, AngularYawVelocity);
        this.robotRelativeSpeeds = robotRelativeSpeeds;
        this.fieldRelativeSpeeds = fieldRelativeSpeeds;
    }

    public Optional<Pose2d> getFieldToRobot(double timestamp) {
        return poseBuffer.getSample(timestamp);
    }

    public Optional<Double> getAngularYawVelocity(double minTime, double maxTime) {
       var submap = AngularYawVelocity.getInternalBuffer().subMap(minTime, maxTime).values();
        var max = submap.stream().max(Double::compare);
        var min = submap.stream().min(Double::compare);
        if (max.isEmpty() || min.isEmpty())
            return Optional.empty();
        if (Math.abs(max.get()) >= Math.abs(min.get()))
            return max;
        else
            return min;
    }

    public void addVisionFieldPoseEstimate(VisionFieldPoseEstimate visionFieldPoseEstimate) {
        visionFieldPoseEstimateConsumer.accept(visionFieldPoseEstimate);
    }

    public Pose2d getLatestFieldToRobot(){
        return poseBuffer.getInternalBuffer().lastEntry().getValue();
    }

    public ChassisSpeeds getFieldRelativeSpeeds(){
        return fieldRelativeSpeeds;
    }

    public ChassisSpeeds getRobotRelativeSpeeds(){
        return robotRelativeSpeeds;
    }

    public boolean isRedAlliance(){
        return DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().equals(Optional.of(Alliance.Red));
    }
}
