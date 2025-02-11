package frc.robot.subsystem.vision;

import java.util.HashSet;
import java.util.List;
import java.util.Optional;
import java.util.Set;

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
import frc.robot.Robot;

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
        //updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front"), LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front"),"limelight-front" );
        //updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back"), LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back"), "limelight-back");
        
        updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-elevato"), LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-elevato"), "limelight-elevato");
        LimelightHelpers.SetRobotOrientation("limelight-elevato",visionState.getPigeonYaw(),0,0,0,0,0);        

        updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-front"), LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-front"), "limelight-front");
        LimelightHelpers.SetRobotOrientation("limelight-front",visionState.getPigeonYaw(),0,0,0,0,0);        

        updateVision(LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight-back"), LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight-back"), "limelight-back");
        LimelightHelpers.SetRobotOrientation("limelight-back",visionState.getPigeonYaw(),0,0,0,0,0);        
        
        
        //LimelightHelpers.SetRobotOrientation(LimelightName,visionState.getPigeonYaw(),0,0,0,0,0);        
        //LimelightHelpers.SetRobotOrientation(LimelightName,visionState.getPigeonYaw(),0,0,0,0,0);        
    }

    public void updateVision(LimelightHelpers.PoseEstimate megaTagPose, LimelightHelpers.PoseEstimate megaTag2Pose, String LimelightName){
        //Double timeisNull = megaTagPose.timestampSeconds;
        if(megaTagPose == null || megaTag2Pose == null) {return ;}
        var updateTimeStamp = megaTagPose.timestampSeconds;
        boolean alreadyProccessed = false;
        switch(LimelightName){
            case "limelight-front":
                if(updateTimeStamp == lastTimeStampFront){
                    alreadyProccessed = true;
                }
                lastTimeStampFront = updateTimeStamp;
                break;
            case "limelight-back":
                if(updateTimeStamp == lastTimeStampBack){
                    alreadyProccessed = true;
                }
                lastTimeStampBack = updateTimeStamp;
                break;
            case "limelight-elevato":
                if(updateTimeStamp == lastTimeStampElevator){
                    alreadyProccessed = true;
                }
                lastTimeStampElevator = updateTimeStamp;
                break;
        }

        if(!alreadyProccessed){
            Optional<VisionFieldPoseEstimate> megaTagEstimate = processMegaTagPoseEst(megaTagPose, LimelightName);
            Optional<VisionFieldPoseEstimate> megaTag2Estimate = processMegaTag2PoseEst(megaTag2Pose, LimelightName);
        

            boolean usedMegaTag = false;
            boolean usedMegaTag2 = false;
                if(megaTagEstimate.isPresent()){
                    if(shouldUseMegatag(megaTagPose)){
                    usedMegaTag = true;
                    usedMegaTag2 = false;
                    visionState.addVisionFieldPoseEstimate(megaTagEstimate.get());
                    System.out.println(usedMegaTag);
                    }
                }
            
                if(megaTag2Estimate.isPresent() && !usedMegaTag){
                    if(shouldUseMegatag2(megaTag2Pose) ){
                        usedMegaTag = false;
                        usedMegaTag2 = true;
                        visionState.addVisionFieldPoseEstimate(megaTag2Estimate.get());
                    }
                }
            SmartDashboard.putBoolean("use megatag", usedMegaTag);
            SmartDashboard.putBoolean("should useMegatag", shouldUseMegatag(megaTagPose));
            SmartDashboard.putBoolean("used MT2", usedMegaTag2);

        }
    } 

    
    public boolean shouldUseMegatag(LimelightHelpers.PoseEstimate megaTagPose){
        double timeStamp = megaTagPose.timestampSeconds;
        var angularYawVelo = visionState.getGreatestAngularYawVelocity(timeStamp - 0.1, timeStamp);
        if(angularYawVelo.isPresent() && Math.abs(angularYawVelo.get())>Units.degreesToRadians(200)){
            return false;
        }
        if(megaTagPose.avgTagArea < VisionConstants.MIN_AREA_FOR_MEGATAG){
            return false;
        }
        if(megaTagPose.rawFiducials.length < 1){
            return false;
        }
        if(megaTagPose.pose.getTranslation().getNorm() > 1.0){
            return false;
        }
        for(RawFiducial fiducial : megaTagPose.rawFiducials){
            if(fiducial.ambiguity < 0.8){
                return false;
            }
        }

        return true;
    }

    public boolean shouldUseMegatag2(LimelightHelpers.PoseEstimate megaTagPose){
        
        double timeStamp = megaTagPose.timestampSeconds;
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
        if(estFieldToRobot.isEmpty()) return Optional.empty();
        double poseDifference = estFieldToRobot.get().getTranslation().getDistance(realFieldToRobot.get().getTranslation());
        if(poseEstimate.rawFiducials.length  > 0){
            double xyStd = 2.0;  
            if(poseEstimate.rawFiducials.length >= 2 && poseEstimate.avgTagArea > 0.1){
                xyStd = 0.1;
            }else if(seesReefTag(poseEstimate.rawFiducials) && poseEstimate.avgTagArea > 0.4){
                xyStd = 0.34;
            }else if (poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5) {
                xyStd = 0.5;
            }else if (poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3) {
                xyStd = 1.0;
            }else if (poseEstimate.rawFiducials.length > 1) {
                xyStd = 1.2;
            }
            Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(xyStd, xyStd, Units.degreesToRadians(50.0));
            Pose2d fieldToRobotEstimate = new Pose2d(estFieldToRobot.get().getTranslation(), realFieldToRobot.get().getRotation());
            return Optional.of(new VisionFieldPoseEstimate(fieldToRobotEstimate, timeStamp, visionMeasurementStdDevs));
        }
        return Optional.empty();
    }

    public Optional<VisionFieldPoseEstimate> processMegaTagPoseEst(LimelightHelpers.PoseEstimate poseEstimate,String LimelightName){
        double timeStamp = poseEstimate.timestampSeconds;
        var realFieldToRobot = visionState.getFieldToRobot(timeStamp);
        if(realFieldToRobot.isEmpty()) return Optional.empty();
        var estFieldToRobot = getFieldToRobot(poseEstimate, LimelightName);
        if(estFieldToRobot.isEmpty()) return Optional.empty();
        double poseDifference = estFieldToRobot.get().getTranslation().getDistance(realFieldToRobot.get().getTranslation());
        if(poseEstimate.rawFiducials.length  > 0){
            double xyStd = 1.0;
            double degStd = 12;
            if(poseEstimate.rawFiducials.length >= 2){
                xyStd = 0.5;
                degStd = 6;
            }else if(poseEstimate.avgTagArea > 0.8 && poseDifference < 0.5){
                xyStd = 1;
                degStd = 12;
            }else if(poseEstimate.avgTagArea > 0.1 && poseDifference < 0.3){
                xyStd = 2;
                degStd = 30;
            }
            Matrix<N3,N1> visionMeasurementStdDevs = VecBuilder.fill(xyStd, xyStd, Units.degreesToRadians(degStd));
            return Optional.of(new VisionFieldPoseEstimate(poseEstimate.pose, timeStamp, visionMeasurementStdDevs));
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
