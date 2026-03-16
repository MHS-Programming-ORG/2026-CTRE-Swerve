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

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.DeferredCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.RossIdleCommand;
import frc.robot.commands.RossShootCommand;
import frc.robot.commands.IntakePivotCommands.AgitatePivotCommand;
import frc.robot.commands.IntakePivotCommands.MoveToPositionMagicCommand;
import frc.robot.commands.IntakeRollerCommands.runIntakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.ArduCam;
import frc.robot.subsystems.ArduCams;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use velocity control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()
    .withHeadingPID(5,0,0)// Placeholder
    .withDeadband(MaxSpeed * 0.1)
    .withRotationalDeadband(0); // Let the PID handle the small movements

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final ArduCam camera = new ArduCam();  
    public final ArduCams cameras = new ArduCams();
    public final PIDController pid = new PIDController(5, 0.0, 0.0);

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(20);
    private final PivotSubsystem m_intakePivot = new PivotSubsystem(19, 0);
    private final ConveyorSubsystem m_ConveyorSubsystem = new ConveyorSubsystem(18);
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(cameras, 15, 16, 17);

  

    SendableChooser<Command> autoChooser;

    Trigger conveyorRunning = new Trigger(() -> m_ConveyorSubsystem.isRunning());

    // double turnOuput = pid.calculate(camera.getYaw(), 0);
    
    //double hubPoseX = (11.920+0.5969);
    //double hubPoseX = DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? (11.920+0.5969) : (4.635-0.5969);
    //double hubPoseY = 0;
    //double hubPoseY = 4.040; 


    public RobotContainer() {
        registerNamedCommands();

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);
        SmartDashboard.putBoolean("Conveyor Trigger", conveyorRunning.getAsBoolean());

        CameraServer.startAutomaticCapture();
        

        pid.enableContinuousInput(-Math.PI, Math.PI);
        configureBindings();
    }

     public void registerNamedCommands(){
        NamedCommands.registerCommand("Intake", new runIntakeCommand(m_intakeSubsystem, m_intakePivot, m_ConveyorSubsystem, shooterSubsystem));
        NamedCommands.registerCommand("IntakePivotDown", new MoveToPositionMagicCommand(m_intakePivot, 22, 0.3));
        NamedCommands.registerCommand("IntakePivotTuck", new MoveToPositionMagicCommand(m_intakePivot, 0, 0.3));
        NamedCommands.registerCommand("Shoot", new RossShootCommand(shooterSubsystem, m_ConveyorSubsystem, 1, 55, 0.6, () -> drivetrain.calculateDistance()));
        NamedCommands.registerCommand("Agitate", new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
        
        NamedCommands.registerCommand("Override Rotation", new InstantCommand(() -> 
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
        return pid.calculate(drivetrain.getPose2d().getRotation().getDegrees(), drivetrain.calculateHubAngle());
        }))
        );
    
        
     }

    

    private void configureBindings() {
        shooterSubsystem.setDefaultCommand(new RossIdleCommand(shooterSubsystem, 20));
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
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
        //double hubPoseX = 0;//DriverStation.getAlliance().orElse(Alliance.Red) == Alliance.Blue ? (11.920+0.5969):

        // joystick.a().whileTrue(
        //      drivetrain.applyRequest(() ->
        //         drive.withVelocityX(joystick.getLeftY() * MaxSpeed*0.1) // Drive forward with negative Y (forward)
        //             .withVelocityY(joystick.getLeftX() * MaxSpeed*0.1) // Drive left with negative X (left)
        //             .withRotationalRate(pid.calculate(
        //                 drivetrain.getPose2d().getRotation().getRadians(),
        //                 drivetrain.calculateAngle(
        //                     hubPoseX,
        //                     hubPoseY
        //                 ))) // Drive counterclockwise with negative X (left)
        //     )
        // );

        joystick.rightTrigger().whileTrue(
            drivetrain.applyRequest(() -> 
                driveFacing
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromRadians(drivetrain.calculateHubAngle()))));

        joystick.a().whileTrue(
            drivetrain.applyRequest(() -> 
                driveFacing
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromRadians(drivetrain.calculatePassAngle()))));
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.b().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

  
    // joystick.b().onTrue(
    //     AutoBuilder.pathfindToPose( // used to test after DriveF2Meters on red side
    //     new Pose2d(13.7, 5.0, Rotation2d.fromDegrees(180)), 
    //     constraints,
    //     0.0 // Goal end velocity
    // ));

    //     joystick.y().onTrue(
    //     AutoBuilder.pathfindToPose(
    //     new Pose2d(15.0, 1.0, Rotation2d.fromDegrees(180)), 
    //     constraints, 
    //     0.0 // Goal end velocity
    // ));
        
    //     joystick.x().onTrue(
    //     AutoBuilder.pathfindToPose(
    //     new Pose2d(12.5, 1.0, Rotation2d.fromDegrees(180)), 
    //     constraints, 
    //     0.0 // Goal end velocity
    // ));


    ////////////////////////////////////
    ///           OPERATOR           ///
    /// ////////////////////////////////
    /// 
    // Shooting with Agitate
    joystick.rightBumper().whileTrue(new runIntakeCommand(m_intakeSubsystem, m_intakePivot, m_ConveyorSubsystem, shooterSubsystem));
    joystick.rightBumper().whileFalse(new InstantCommand(() -> m_intakeSubsystem.setSpeed(0)));
    
    //Intaking with the Conveyor
    joystick.leftBumper().whileTrue(new RossShootCommand(shooterSubsystem, m_ConveyorSubsystem, 1, 55, 0.5, () -> drivetrain.calculateDistance()));
    joystick.leftBumper().and(conveyorRunning).whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
    // joystick.leftBumper().whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
    
    //Passing
    joystick.leftTrigger().whileTrue(new RossShootCommand(shooterSubsystem, m_ConveyorSubsystem, 1, 55, 0.5, () -> 3.9624));
    // joystick.leftBumper().whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem)); 
    

    // joystick.x().onTrue(new InstantCommand(() -> cameras.driveModeOn()));
    // joystick.y().onTrue(new InstantCommand(() -> cameras.driveModeOff()));

    //Zeroing the pivot of the intake
    joystick.y().onTrue(new MoveToPositionMagicCommand(m_intakePivot, 0, 0.1));
    joystick.x().whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
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

    public ArduCams getCameras(){
        return cameras;
    }

    public PivotSubsystem getPivotSubsystem(){
        return m_intakePivot;
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

}
