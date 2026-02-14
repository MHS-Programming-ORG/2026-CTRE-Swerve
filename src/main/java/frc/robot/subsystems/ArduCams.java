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
    boolean camera1HasResult = false;
    boolean camera2HasResult = false;
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


    public boolean cameraHasTargets(PhotonCamera camera){
        if(camera.equals(camera1)){
            return camera1HasResult;
        }else{
            return camera2HasResult;
        }
        
    }

    public Pose2d getEstimatedPose(){
        return estimate.estimatedPose.toPose2d();
    }

    public double getTimestampSeconds(){
        return estimate.timestampSeconds;
    }

    public void getResults(PhotonTrackedTarget target, PhotonPipelineResult result){
        if (result.hasTargets()) {
            target = result.getBestTarget();
            int targetID = target.getFiducialId();
            //double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            double yaw = target.getYaw();
            double pitch = target.getPitch();
            double skew = target.getSkew();
            
            SmartDashboard.putNumber("yaw", yaw);
            SmartDashboard.putNumber("pitch", pitch);
            SmartDashboard.putNumber("skew", skew);
            
            SmartDashboard.putNumber("Target X", bestCameraToTarget.getX());
            SmartDashboard.putNumber("Target Y", bestCameraToTarget.getY());
            SmartDashboard.putNumber("Target Z", bestCameraToTarget.getZ());
        }
    }
    
    public void getEstimates(PhotonPoseEstimator estimator, PhotonPipelineResult result){
        Optional<EstimatedRobotPose> visionEst = estimator.estimateCoprocMultiTagPose(result);

        if (visionEst.isEmpty() && result.hasTargets()){
            visionEst = estimator.estimateLowestAmbiguityPose(result);
        }

        if (visionEst.isPresent()) {
        }
            SmartDashboard.putString("visionEst", result.toString());
            estimate = visionEst.get();

            double estX = estimate.estimatedPose.getMeasureX().baseUnitMagnitude();
            double estY = estimate.estimatedPose.getMeasureY().baseUnitMagnitude();
            double estZ = estimate.estimatedPose.getMeasureZ().baseUnitMagnitude();
            double estTheta = estimate.estimatedPose.getRotation().getAngle() * (180/Math.PI);

            SmartDashboard.putNumber("estimatedX", estX);
            SmartDashboard.putNumber("estimatedY", estY);
            SmartDashboard.putNumber("estimatedZ", estZ);
            SmartDashboard.putNumber("estimatedAngle", estTheta);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result1 = camera1.getLatestResult();
        PhotonPipelineResult result2 = camera2.getLatestResult();
        camera1HasResult = result1.hasTargets();
        camera2HasResult = result2.hasTargets();

        getEstimates(photonEstimator1, result1);
        getEstimates(photonEstimator2, result2);
        
        getResults(target1, result1);
        getResults(target2, result2);



        SmartDashboard.putBoolean(
            "NT Connected",
            NetworkTableInstance.getDefault().isConnected()
        );

        SmartDashboard.putBoolean("Camera1 Has Result", camera1HasResult);
        SmartDashboard.putBoolean("Camera2 Has Result", camera2HasResult);


        
    }
}