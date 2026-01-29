package frc.robot.subsystems;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ArduCam extends SubsystemBase{

    private CommandSwerveDrivetrain swerve;

    public ArduCam(CommandSwerveDrivetrain swerve)
    {
        this.swerve = swerve;
    }

    // http://photonvision.local:5800/
    PhotonCamera camera = new PhotonCamera("Arducam_OV9782_USB_Camera");
    PhotonTrackedTarget target = camera.getLatestResult().getBestTarget();
    boolean hasResult = false;
    EstimatedRobotPose estimate;

    public static final AprilTagFieldLayout kTagLayout =
                AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

    // // https://docs.photonvision.org/en/latest/docs/apriltag-pipelines/coordinate-systems.html#coordinate-systems
    public static final Transform3d kRobotToCam =
                new Transform3d(new Translation3d(0.365, 0.004, 0.0), new Rotation3d(0, 0, 0));

     PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, kRobotToCam);

    double yaw = 0;
    
    public double getYaw(){
        return yaw;
    }

    public boolean cameraHasTargets(){
        return hasResult;
    }

    public void updatePoseWithVision(CommandSwerveDrivetrain swerve){
        swerve.addVisionMeasurement(estimate.estimatedPose.toPose2d(), estimate.timestampSeconds);
    }

    @Override
    public void periodic() {
        PhotonPipelineResult result = camera.getLatestResult();
        hasResult = result.hasTargets();

        Optional<EstimatedRobotPose> visionEst = photonEstimator.estimateCoprocMultiTagPose(result);
        if (visionEst.isEmpty()) {
            visionEst = photonEstimator.estimateLowestAmbiguityPose(result);
        } else{
            SmartDashboard.putString("visionEst", result.toString());
            estimate = visionEst.get();

            double estX = estimate.estimatedPose.getMeasureX().baseUnitMagnitude();
            double estY = estimate.estimatedPose.getMeasureY().baseUnitMagnitude();
            double estZ = estimate.estimatedPose.getMeasureZ().baseUnitMagnitude();
            double estTheta = estimate.estimatedPose.getRotation().getAngle() * (Math.PI / 180);

            SmartDashboard.putNumber("estimatedX", estX);
            SmartDashboard.putNumber("estimatedY", estY);
            SmartDashboard.putNumber("estimatedZ", estZ);
            SmartDashboard.putNumber("estimatedAngle", estTheta);

            updatePoseWithVision(swerve);
        }
        
 
        
        

        SmartDashboard.putBoolean(
            "NT Connected",
            NetworkTableInstance.getDefault().isConnected()
        );

        SmartDashboard.putBoolean(
            "Photon Camera Connected",
            camera.isConnected()
        );

        SmartDashboard.putNumber(
            "Photon Timestamp",
            result.getTimestampSeconds()
        );
        SmartDashboard.putBoolean(
            "Photon Has Target",
            hasResult
        );
        SmartDashboard.putNumber(
            "Photon Target Count",
            result.getTargets().size()
        );

        if (result.hasTargets()) {
            target = result.getBestTarget();
            int targetID = target.getFiducialId();
            double poseAmbiguity = target.getPoseAmbiguity();
            Transform3d bestCameraToTarget = target.getBestCameraToTarget();
            yaw = target.getYaw();
            double pitch = target.getPitch();
            double area = target.getArea();
            double skew = target.getSkew();
            
            SmartDashboard.putNumber("yaw", yaw);
            SmartDashboard.putNumber("pitch", pitch);
            SmartDashboard.putNumber("skew", skew);
            
            SmartDashboard.putNumber("Target X", bestCameraToTarget.getX());
            SmartDashboard.putNumber("Target Y", bestCameraToTarget.getY());
            SmartDashboard.putNumber("Target Z", bestCameraToTarget.getZ());
        }
    }
}