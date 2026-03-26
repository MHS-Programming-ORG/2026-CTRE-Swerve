package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;


public class ArduCams extends SubsystemBase{

    // http://photonvision.local:5800/
    PhotonCamera camera1 = new PhotonCamera("BlueSaber2"); // BlueSaber1
    //PhotonCamera camera2 = new PhotonCamera("BlueSaber1");
    PhotonPoseEstimator photonPoseEstimator1;
    PhotonPoseEstimator photonPoseEstimator2;
    PhotonTrackedTarget target1 = camera1.getLatestResult().getBestTarget();
    //PhotonTrackedTarget target2 = camera2.getLatestResult().getBestTarget();
    private double getx = 0;
    private double anf = 0;
    
    public final AprilTagFieldLayout kTagLayout = 
                AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltWelded);
    
                // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#coordinate-systems
    public final Transform3d kRobotToCam1 =
                new Transform3d(new Translation3d(-0.343, 0.315, 0.0), new Rotation3d(0, -20*Math.PI/180, Math.PI));

    public final Transform3d kRobotToCam2 = 
                new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0));

    PhotonPoseEstimator photonEstimator1;
    PhotonPoseEstimator photonEstimator2;

    public ArduCams(){
        photonEstimator1 = new PhotonPoseEstimator(kTagLayout, kRobotToCam1);
        photonEstimator2 = new PhotonPoseEstimator(kTagLayout, kRobotToCam2);
    }
    
    EstimatedRobotPose estimate;

   

    public Optional<EstimatedRobotPose> getEstimatedPose(PhotonPoseEstimator estimator, PhotonCamera camera){
    PhotonPipelineResult result = camera.getLatestResult();
    if(result.targets.size() >= 2){
        return estimator.estimateCoprocMultiTagPose(result);
    } else if (result.targets.size() == 1){
        return estimator.estimateLowestAmbiguityPose(result);
    }
    return Optional.empty();
    }

    public Optional<EstimatedRobotPose> getEstimatedPoseCam1(){
        return getEstimatedPose(photonEstimator1, camera1);
    }

    // public Optional<EstimatedRobotPose> getEstimatedPoseCam2(){
    //     return getEstimatedPose(photonEstimator2, camera2);
    // }

    public void driveModeOn(){
        camera1.setDriverMode(true);
      //  camera2.setDriverMode(true);
    }

    public void driveModeOff(){
        camera1.setDriverMode(false);
      //  camera2.setDriverMode(false);
    }

    double hubPoseX = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 11.920:4.630;
    //double hubPoseY = 0;
    double hubPoseY = 4.040;

    //////////////////////////////////////////////////////////////////////////////////////// davek shenannigans
    public double getX(Double distance){
        PhotonPipelineResult result = camera1.getLatestResult();
        double y = 0;
        if(result.hasTargets()){
            for(var target : result.getTargets()){
                int id = target.getFiducialId();
                Transform3d trandsfomr = target.getBestCameraToTarget();
                double x = trandsfomr.getTranslation().getX();
                double z = trandsfomr.getTranslation().getZ();
                anf = Math.toDegrees(Math.atan2(x,z));

                if(id == 9 || id == 10 || id == 25 || id == 26){
                    y = target.getBestCameraToTarget().getX();
                   // y = hubMath(id, target.getBestCameraToTarget().getX(), rotation3d.getZ());
                    break; // stop once we find a valid tag
                }
            }
        }

        //double y = result.hasTargets()?result.getBestTarget().getBestCameraToTarget().getX():0;
        if(y != 0){getx = y;}else{getx = distance;} // set distance to your fixed RPS when april tag is undetected]]]]]]]]]]]
        SmartDashboard.putBoolean("UsingCamera", y != 0);
        return getx;
    }

    public double hubMath(int id, double distance, double angle){
        double dist = distance;
        if(id == 11 || id == 27 || id == 9 || id == 25  || id == 8 || id == 24){
            dist = Math.sqrt(Math.pow(dist, 2)+Math.pow(14.94, 2) - 2*dist*14.94*Math.cos(angle));
        }
        return Math.sqrt(Math.pow(dist, 2)+Math.pow(23.5, 2) - 2*dist*23.5*Math.cos(angle));
    }

    public boolean cameraVisable(){
        return camera1.getLatestResult().hasTargets();
    }
    ////////////////////////////////////////////////////////////////////////////////////////


    // @Override
    public void periodic() {
        //SmartDashboard.putNumber("xx", anf);
        
        // if (camera1.hasTargets()) {
        //     PhotonPipelineResult target = camera1.getLatestResult();
        //     Rotation3d rotation = target.getRotation();
        //     double yaw = rotation.getYaw();
        //     double pitch = rotation.getPitch();
            
        //     SmartDashboard.putNumber("yaw", yaw);
        //     SmartDashboard.putNumber("pitch", pitch);
        // }
        
        
        // if (getEstimatedPoseCam1().isPresent()){
        //     EstimatedRobotPose pose1 = getEstimatedPoseCam1().get();
        //     SmartDashboard.putNumber("Camera1EstX", pose1.estimatedPose.getX());
        //     SmartDashboard.putNumber("Camera1EstY", pose1.estimatedPose.getY());
        //     SmartDashboard.putNumber("Camera1EstRotation", pose1.estimatedPose.getRotation().getAngle());
        // } 
        
       
        // if (getEstimatedPoseCam2().isPresent()){
        //      EstimatedRobotPose pose2 = getEstimatedPoseCam2().get();
        //     SmartDashboard.putNumber("Camera2EstX", pose2.estimatedPose.getX());
        //     SmartDashboard.putNumber("Camera2EstY", pose2.estimatedPose.getY());
        //     SmartDashboard.putNumber("Camera2EstRotation", pose2.estimatedPose.getRotation().getAngle());
        // }
        
    }
}