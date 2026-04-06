// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.FixedShootCommand;
import frc.robot.commands.RossIdleCommand;
import frc.robot.commands.RossShootCommand;
import frc.robot.commands.IntakePivotCommands.AgitatePivotCommand;
import frc.robot.commands.IntakePivotCommands.MoveToPositionMagicCommand;
import frc.robot.commands.IntakeRollerCommands.runIntakeCommand;
import frc.robot.commands.IntakeRollerCommands.runOuttakeCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.NetworkingPython;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
// import frc.robot.subsystems.ArduCam;
import frc.robot.subsystems.ArduCams;
import frc.robot.commands.CloseShootCommand;


public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use velocity control for drive motors

    private final SwerveRequest.FieldCentricFacingAngle driveFacing = new SwerveRequest.FieldCentricFacingAngle()    
    .withHeadingPID(5,0,0)// Placeholder
    .withDeadband(MaxSpeed * 0.1);

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    // public final ArduCam camera = new ArduCam();  
    public final ArduCams cameras = new ArduCams();
    public final PIDController pid = new PIDController(5, 0.0, 0.0);

    private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem(20, 19); //FIXME add intake follower ID
    private final PivotSubsystem m_intakePivot = new PivotSubsystem(21, 0);
    private final ConveyorSubsystem m_ConveyorSubsystem = new ConveyorSubsystem(17, 18); //FIXME add indexer ID
    private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(cameras, 15, 16, 14);
    private final NetworkingPython networkingPython = new NetworkingPython();

    SendableChooser<Command> autoChooser;

    Trigger conveyorRunning = new Trigger(() -> m_ConveyorSubsystem.isRunning());
    Trigger outtakeTrigger = new Trigger(() -> networkingPython.outtakePressed());
    // Trigger agitateTrigger = new Trigger(() -> networkingPython.agitatePressed());
    Trigger weWinTrigger = new Trigger(() -> networkingPython.weWinPressed());
    Trigger fastRevTrigger = new Trigger(() -> networkingPython.fastRevPressed());

    Trigger inAllianceZoneTrigger = new Trigger(() -> 
        drivetrain.getPose2d().getX() <= 4.625 || 
        drivetrain.getPose2d().getX() >= 11.915
        );

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

        // CameraServer.startAutomaticCapture();
        

        pid.enableContinuousInput(-Math.PI, Math.PI);
        configureBindings();
    }

     public void registerNamedCommands(){
        NamedCommands.registerCommand("Intake", new runIntakeCommand(m_intakePivot, m_intakeSubsystem));
        NamedCommands.registerCommand("IntakePivotDown", new MoveToPositionMagicCommand(m_intakePivot, 28, 0.3));
        NamedCommands.registerCommand("IntakePivotTuck", new MoveToPositionMagicCommand(m_intakePivot, 0, 0.3));
        NamedCommands.registerCommand("Shoot", new RossShootCommand(shooterSubsystem, m_ConveyorSubsystem, 70, 0.7, () -> drivetrain.calculateDistance()));
        NamedCommands.registerCommand("Agitate", new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
        NamedCommands.registerCommand("TrenchSHOT", new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 50, 0.5, 60));
        NamedCommands.registerCommand("MidishSHOT", new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 50, 0.5, 45));
        
        NamedCommands.registerCommand("AimToHub", drivetrain.applyRequest(() -> 
                driveFacing
                .withTargetDirection(Rotation2d.fromRadians(drivetrain.calculateHubAngle()))));

    
        
     }

    

    private void configureBindings() {
        outtakeTrigger.whileTrue(new runOuttakeCommand(m_intakeSubsystem, m_intakePivot, m_ConveyorSubsystem));
        // agitateTrigger.whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));

        // shooterSubsystem.setDefaultCommand(new RossIdleCommand(shooterSubsystem, 30)); //FIXME change back
        //shooterSubsystem.getShooterShoot(drivetrain.calculateDistance()) + 10;
        // fastRevTrigger.whileTrue(new RossIdleCommand(shooterSubsystem, 90, 0));
        // inAllianceZoneTrigger.whileTrue(new RossIdleCommand(shooterSubsystem, 90, 0));
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



        joystick.b().whileTrue(
            drivetrain.applyRequest(() -> 
                driveFacing
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromRadians(drivetrain.calculateHubAngle()))));
        joystick.b().whileTrue(new RossIdleCommand(shooterSubsystem, 90, 0));

        joystick.y().whileTrue(
            drivetrain.applyRequest(() -> 
                driveFacing
                .withVelocityX(-joystick.getLeftY() * MaxSpeed)
                .withVelocityY(-joystick.getLeftX() * MaxSpeed)
                .withTargetDirection(Rotation2d.fromRadians(drivetrain.calculatePassAngle()))));

        joystick.start().whileTrue(new runIntakeCommand(m_intakePivot, m_intakeSubsystem));
        joystick.start().whileTrue(new RossShootCommand(shooterSubsystem, m_ConveyorSubsystem, MaxSpeed, MaxAngularRate, () -> drivetrain.calculateDistance()));
        
        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        // joystick.x().whileTrue(new InstantCommand(() -> shooterSubsystem.setVoltage(6)));
        // joystick.x().whileFalse(new InstantCommand(() -> shooterSubsystem.setVoltage(0.0)));
        joystick.x().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        // drivetrain.registerTelemetry(logger::telemeterize);

  
    // joystick.b().onTrue(
    //     AutoBuilder.pathfindToPose( // used to
    // test after DriveF2Meters on red side
    //     new Pose2d(13.7, 5.0, Rotation2d.fromDegrees(180)), 
    //     constraints,
    //     0.0 // Goal end velocity
    // ));


    ////////////////////////////////////
    ///           OPERATOR           ///
    /// ////////////////////////////////
    /// 

    // Shooting with Agitate
    joystick.rightBumper().whileTrue(new runIntakeCommand(m_intakePivot, m_intakeSubsystem));
    joystick.rightBumper().whileFalse(new InstantCommand(() -> m_intakeSubsystem.setSpeed(0)));
    // joystick.rightBumper().whileTrue(
    //     drivetrain.applyRequest(() ->
    //             drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * 0.26385224274) 
    //                 .withVelocityY(-joystick.getLeftX() * MaxSpeed * 0.26385224274)
    //                 .withRotationalRate(-joystick.getRightX() * MaxAngularRate)
    //         ));
    //multiply by 0.26385224274 to allegedly drive 1 m/s
    
    
    //Shooting
    // joystick2.a().whileTrue(new InstantCommand(() -> m_ConveyorSubsystem.setConveyorSpeed(0.3)));//FIXME TESTING
    joystick.leftBumper().whileTrue(new RossShootCommand(shooterSubsystem, m_ConveyorSubsystem, 70, 0.7, () -> drivetrain.calculateDistance()));    
    joystick.leftBumper().and(conveyorRunning).whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));

    //Pit Check 
    //joystick.leftBumper().whileTrue(new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 10, 0.1, 10));
    // joystick.leftBumper().whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));

    //Trench Shooting
    joystick.rightTrigger().whileTrue(new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 50, 0.5, 60));
    joystick.rightTrigger().and(conveyorRunning).whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
    
    //Tower Shooting
    joystick.leftTrigger().whileTrue(new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 60, 0.6, 80));
    joystick.leftTrigger().and(conveyorRunning).whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
    
    //Passing
    joystick.a().whileTrue(new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 80, 0.8, 60));
    joystick.a().and(conveyorRunning).whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));

    // joystick2.b().whileTrue(new InstantCommand(() -> m_intakeSubsystem.setSpeed(0.5)));
    // joystick2.b().whileFalse(new InstantCommand(() -> m_intakeSubsystem.setSpeed(0)));


    // joystick2.a().onTrue(new MoveToPositionMagicCommand(m_intakePivot, 28, 0.1));
    // joystick2.b().onTrue(new MoveToPositionMagicCommand(m_intakePivot, 0, 0.1));

    
    // joystick2.x().whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));
    // joystick2.x().whileFalse(new InstantCommand(() -> shooterSubsystem.setVoltage(0.0)));
    // joystick2.a().whileFalse(new InstantCommand(() -> m_intakePivot.setVoltage(0)))

    //Near Hub Shooting
    joystick.back().whileTrue(new FixedShootCommand(shooterSubsystem, m_ConveyorSubsystem, 80, 0.6, 40));
    joystick.back().whileTrue(new AgitatePivotCommand(m_intakePivot, m_intakeSubsystem));

    }


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
