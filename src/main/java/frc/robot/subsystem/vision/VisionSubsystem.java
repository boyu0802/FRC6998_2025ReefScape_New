package frc.robot.subsystem.vision;

import static frc.robot.RobotMap.LIMELIGHT_ELEVATOR;
import static frc.robot.RobotMap.LIMELIGHT_LEFT;
import static frc.robot.RobotMap.LIMELIGHT_RIGHT;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.limelight.LimelightHelpers;
import frc.lib.limelight.LimelightHelpers.RawFiducial;
import frc.robot.Constants.HangConstants.VisionConstants;

import static frc.robot.Constants.PhotonAprilTagConstants.*;


public class VisionSubsystem extends SubsystemBase{
    private final VisionState visionState;
    private double lastTimeStampFront = 0.0;
    private double lastTimeStampBack = 0.0;
    private double lastTimeStampElevator = 0.0;
    private final static Set<Integer> BLUE_REEF_TAGS = new HashSet<>(List.of(17,18,19,20,21,22));
    private final static Set<Integer> RED_REEF_TAGS = new HashSet<>(List.of(6,7,8,9,10,11));
    private Set<Integer> currentReefTags;


    public VisionSubsystem(VisionState visionState){
        this.visionState = visionState;
        currentReefTags = visionState.isRedAlliance() ? RED_REEF_TAGS : BLUE_REEF_TAGS;
        
       
    }

    @Override
    public void periodic(){
                
        updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_ELEVATOR), LIMELIGHT_ELEVATOR);
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_ELEVATOR,visionState.getPigeonYaw(),0,0,0,0,0);        

        updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_LEFT), LIMELIGHT_LEFT);
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_LEFT,visionState.getPigeonYaw(),0,0,0,0,0);        

        updateVision( LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_RIGHT), LIMELIGHT_RIGHT);
        LimelightHelpers.SetRobotOrientation(LIMELIGHT_RIGHT,visionState.getPigeonYaw(),0,0,0,0,0);      


        // Logger.recordOutput("Vision/LIMELIGHT_RIGHT/Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_RIGHT).pose);
        // Logger.recordOutput("Vision/LIMELIGHT_LEFT/Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_LEFT).pose);
        // Logger.recordOutput("Vision/LIMELIGHT_ELEVATOR/Pose", LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(LIMELIGHT_ELEVATOR).pose);
        //LimelightHelpers.SetRobotOrientation(LimelightName,visionState.getPigeonYaw(),0,0,0,0,0);        
        //LimelightHelpers.SetRobotOrientation(LimelightName,visionState.getPigeonYaw(),0,0,0,0,0);        
    
        
    }

    public void updateVision(LimelightHelpers.PoseEstimate megaTag2Pose, String LimelightName){
        //Double timeisNull = megaTagPose.timestampSeconds;
        if(megaTag2Pose == null) {return ;}
        var updateTimeStamp = megaTag2Pose.timestampSeconds;
        boolean alreadyProccessed = false;
        switch(LimelightName){
            case LIMELIGHT_LEFT:
                if(updateTimeStamp == lastTimeStampFront){
                    alreadyProccessed = true;
                }
                lastTimeStampFront = updateTimeStamp;
                break;
            case LIMELIGHT_RIGHT:
                if(updateTimeStamp == lastTimeStampBack){
                    alreadyProccessed = true;
                }
                lastTimeStampBack = updateTimeStamp;
                break;
            case LIMELIGHT_ELEVATOR:
                if(updateTimeStamp == lastTimeStampElevator){
                    alreadyProccessed = true;
                }
                lastTimeStampElevator = updateTimeStamp;
                break;
        }

        if(!alreadyProccessed){
            //Optional<VisionFieldPoseEstimate> megaTagEstimate = processMegaTagPoseEst(megaTagPose, LimelightName);
            Optional<VisionFieldPoseEstimate> megaTag2Estimate = processMegaTag2PoseEst(megaTag2Pose, LimelightName);
        

            //boolean usedMegaTag = false;
            boolean usedMegaTag2 = false;
                if(megaTag2Estimate.isPresent()){
                    if(shouldUseMegatag2(megaTag2Pose) ){
                        //usedMegaTag = false;
                        usedMegaTag2 = true;
                        Logger.recordOutput(LimelightName, megaTag2Estimate.get().getVisionRobotPoseMeters());
                        visionState.addVisionFieldPoseEstimate(megaTag2Estimate.get());
                    }
                }
            //SmartDashboard.putBoolean("use megatag", usedMegaTag);
            //SmartDashboard.putBoolean("should useMegatag", shouldUseMegatag(megaTagPose));
            SmartDashboard.putBoolean("used MT2", usedMegaTag2);

        }
    }

    public boolean shouldUseMegatag2(LimelightHelpers.PoseEstimate megaTag2Pose){
        
        double timeStamp = megaTag2Pose.timestampSeconds;
        var angularYawVelo = visionState.getGreatestAngularYawVelocity(timeStamp - 0.1, timeStamp);
        if(angularYawVelo.isPresent() && Math.abs(angularYawVelo.get())>Units.degreesToRadians(200)){
                return false;
        }
        return true;
    }

    public Optional<Pose2d> getFieldToRobot(LimelightHelpers.PoseEstimate poseEstimate, String LimelightName){
        var fieldToCamera = poseEstimate.pose;
        if(fieldToCamera.getX() == 0)return Optional.empty();
        // var cameraToRobot = VisionConstants.getCameraToRobot(LimelightName); //TODO: test if the values are pose est of camera or of robot
        // var fieldToRobot = fieldToCamera.plus(cameraToRobot);
        return Optional.of(fieldToCamera); 
    }
    
    //Todo: test all std values
    public Optional<VisionFieldPoseEstimate> processMegaTag2PoseEst(LimelightHelpers.PoseEstimate poseEstimate,String LimelightName){
        double timeStamp = poseEstimate.timestampSeconds;
        var realFieldToRobot = visionState.getFieldToRobot(timeStamp);
        if(realFieldToRobot.isEmpty()) return Optional.empty();
        var estFieldToRobot = getFieldToRobot(poseEstimate, LimelightName);
        //var estStdDevs = kSingleTagStdDevs;
        if(estFieldToRobot.isEmpty()) return Optional.empty();
        double poseDifference = estFieldToRobot.get().getTranslation().getDistance(realFieldToRobot.get().getTranslation());
        
        var estStdDevs = kSingleTagStdDevs;
        
        
        if(poseEstimate.rawFiducials.length  > 0){
            
            
            double xyStdDev = 2.0;  
            double thetaStdDev = Units.degreesToRadians(50.0);
            double avgDist = poseEstimate.avgTagDist;
            


            if(poseEstimate.rawFiducials.length == 1) {
                xyStdDev = xyStdDevModel.predict(avgDist);
                thetaStdDev = thetaStdDevModel.predict(avgDist);
                estStdDevs = VecBuilder.fill(xyStdDev*59.4, xyStdDev*59.4, thetaStdDev*59.4);
            }
            else if(poseEstimate.rawFiducials.length > 1) {
                xyStdDev = Math.pow(avgDist, 2.0) / poseEstimate.rawFiducials.length;
                thetaStdDev = Math.pow(avgDist, 2.0) / poseEstimate.rawFiducials.length;
                estStdDevs = VecBuilder.fill(xyStdDev*59.4, xyStdDev*59.4, thetaStdDev*59.4);
            }
            /* 
            if(poseEstimate.rawFiducials.length >= 2 && poseEstimate.avgTagArea > 0.1){
                xyStdDev = 0.1;
            }else if(seesReefTag(poseEstimate.rawFiducials) && poseEstimate.avgTagArea > 0.4){
                xyStdDev = 0.34;
            }else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStdDev = 0.5;
            }else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStdDev = 1.0;
            }else if (poseEstimate.rawFiducials.length > 1) {
                xyStdDev = 1.2;
            }
                */
            
            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStdDev, xyStdDev, thetaStdDev);
            
            Pose2d fieldToRobotEstimate = new Pose2d(estFieldToRobot.get().getTranslation(), realFieldToRobot.get().getRotation());
            
            return Optional.of(new VisionFieldPoseEstimate(fieldToRobotEstimate, timeStamp, visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    
    
    private boolean seesReefTag(RawFiducial[] fids){
        for(int tag: currentReefTags)        
            for(RawFiducial fid: fids){    
                if(tag == fid.id){
                    return true;
                }
            }

        return false;
    }



    
}
