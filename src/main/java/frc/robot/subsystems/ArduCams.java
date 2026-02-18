package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d; // <--- If this yellow line goes away your doing somthing WRONG
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.PoseEstimator;


public class ArduCams extends SubsystemBase{

    // http://photonvision.local:5800/
    PhotonCamera camera1 = new PhotonCamera("BlueSaber1"); // BlueSaber1
    PhotonCamera camera2 = new PhotonCamera("BlueSaber2");
    PhotonPoseEstimator photonPoseEstimator1;
    PhotonPoseEstimator photonPoseEstimator2;
    PhotonTrackedTarget target1 = camera1.getLatestResult().getBestTarget();
    PhotonTrackedTarget target2 = camera2.getLatestResult().getBestTarget();
    
    public final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
    
                // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#coordinate-systems
    public final Transform3d kRobotToCam1 =
                new Transform3d(new Translation3d(0.365, 0.004, 0.0), new Rotation3d(0, 0, 0));

    public final Transform3d kRobotToCam2 = 
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));


    public ArduCams(){
        photonEstimator1 = new PhotonPoseEstimator(kTagLayout, kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(kTagLayout, kRobotToCam2);
    }
    
    EstimatedRobotPose estimate;

     PhotonPoseEstimator photonEstimator1 = new PhotonPoseEstimator(kTagLayout, kRobotToCam1);
     PhotonPoseEstimator photonEstimator2 = new PhotonPoseEstimator(kTagLayout, kRobotToCam2);

    public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPoseEstimator estimator, PhotonCamera camera){
    PhotonPipelineResult result = camera.getLatestResult();
    if(!result.hasTargets()){
        return Optional.empty();
    }
    return estimator.estimateCoprocMultiTagPose(result);
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseCam1(){
        return getEstimatedPose(photonEstimator1, camera1);
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseCam2(){
        return getEstimatedPose(photonEstimator2, camera2);
    }

    // @Override
    public void periodic() {

        if (getEstimatedPoseCam1().isPresent()){
            EstimatedRobotPose pose1 = getEstimatedPoseCam1().get();
            SmartDashboard.putNumber("Camera1EstX", pose1.estimatedPose.getX());
            SmartDashboard.putNumber("Camera1EstY", pose1.estimatedPose.getY());
            SmartDashboard.putNumber("Camera1EstZ", pose1.estimatedPose.getZ());
        }
        
        if (getEstimatedPoseCam2().isPresent()){
            EstimatedRobotPose pose2 = getEstimatedPoseCam2().get();
            SmartDashboard.putNumber("Camera2EstX", pose2.estimatedPose.getX());
            SmartDashboard.putNumber("Camera2EstY", pose2.estimatedPose.getY());
            SmartDashboard.putNumber("Camera2EstZ", pose2.estimatedPose.getZ());
        } else{
            
        }
        
    }
}