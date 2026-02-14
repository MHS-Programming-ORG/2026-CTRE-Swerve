// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.List;
import java.util.Set;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import frc.robot.subsystems.ArduCam;
import frc.robot.subsystems.ArduCams;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final ArduCam camera = new ArduCam();
    public final ArduCams cameras = new ArduCams();
    public final PIDController pid = new PIDController(0.1, 0.0, 0.0);

    SendableChooser<Command> autoChooser;

    double turnOuput = pid.calculate(camera.getYaw(), 0);

    public RobotContainer() {
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();
    }

    public void registerNamedCommands(){
        NamedCommands.registerCommand("Override Rotation", new InstantCommand(() -> 
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
        return camera.cameraHasTargets() ? pid.calculate(camera.getYaw(), 0) : 0;
        }))
        );
        
    }

    

    private void configureBindings() {
        
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed*0.25) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed*0.25) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        // joystick.a().whileTrue(
        //      drivetrain.applyRequest(() ->
        //         drive.withVelocityX(joystick.getLeftY() * MaxSpeed*0.1) // Drive forward with negative Y (forward)
        //             .withVelocityY(joystick.getLeftX() * MaxSpeed*0.1) // Drive left with negative X (left)
        //             .withRotationalRate(camera.cameraHasTargets() ? pid.calculate(-camera.getYaw(), 0) : -joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        //     )
        // );

        // IN METERES
        double hubPoseX = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 11.920:4.630;
        double hubPoseY = 4.040; 

        joystick.a().whileTrue(
             drivetrain.applyRequest(() ->
                drive.withVelocityX(joystick.getLeftY() * MaxSpeed*0.1) // Drive forward with negative Y (forward)
                    .withVelocityY(joystick.getLeftX() * MaxSpeed*0.1) // Drive left with negative X (left)
                    .withRotationalRate(pid.calculate(
                        drivetrain.getPose2d().getRotation().getRadians(),
                        drivetrain.calculateAngle(
                            hubPoseX,
                            hubPoseY
                        ))) // Drive counterclockwise with negative X (left)
            )
        );
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

  
    joystick.b().onTrue(
        AutoBuilder.pathfindToPose(
        new Pose2d(16, 3.4, Rotation2d.fromDegrees(180)), 
        constraints,
        0.0 // Goal end velocity
    ));

        joystick.y().onTrue(
        AutoBuilder.pathfindToPose(
        new Pose2d(15.0, 1.0, Rotation2d.fromDegrees(180)), 
        constraints, 
        0.0 // Goal end velocity
    ));
        
        joystick.x().onTrue(
        AutoBuilder.pathfindToPose(
        new Pose2d(12.5, 1.0, Rotation2d.fromDegrees(180)), 
        constraints, 
        0.0 // Goal end velocity
    ));

        
    

    }
    
    
    ////////////////////////////////////
    ///       ON-THE-FLY PATHS       ///
    /// ////////////////////////////////
    

    // WAYPOINTS ARE BASED ON DRIVERSTATION TEAM COLOR
    
    PathConstraints constraints = new PathConstraints(1.0/6.0, 1.0/6.0, Math.PI/3 , Math.PI/6); // The constraints for this path.

    ////////////////////////////////////
    ///        GET SUBSYSTEMS        ///
    ////////////////////////////////////
    
    public CommandSwerveDrivetrain getSwerveSubsystem(){
        return drivetrain;
    }
    public ArduCams getCamera(){
        return cameras;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
