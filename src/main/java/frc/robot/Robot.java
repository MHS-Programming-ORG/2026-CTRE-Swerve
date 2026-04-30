// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.HootAutoReplay;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArduCams;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.epilogue.Epilogue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.subsystems.PivotSubsystem;

@Logged
public class Robot extends LoggedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    final CommandSwerveDrivetrain swerve;
    final ArduCams cameras;
    final PivotSubsystem pivot;

    /*
     * private final TalonFX fx1 = new TalonFX(1);
     * private final TalonFX fx2 = new TalonFX(2);
     * private final TalonFX fx3 = new TalonFX(3);
     * private final TalonFX fx4 = new TalonFX(4);
     * private final Orchestra orchestra = new Orchestra();
     * public final AudioConfigs audioConfigs = new AudioConfigs();
     */

    // /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
            .withTimestampReplay()
            .withJoystickReplay();

    public Robot() {

        m_robotContainer = new RobotContainer();
        cameras = m_robotContainer.getCameras();
        swerve = m_robotContainer.getSwerveSubsystem();
        pivot = m_robotContainer.getPivotSubsystem();

        DataLogManager.start();
        Logger.addDataReceiver(new NT4Publisher());
        Logger.start();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run();
        Optional<EstimatedRobotPose> cam1Estimate = cameras.getEstimatedPoseCam1();
        Logger.recordOutput("Robot Pose", m_robotContainer.getSwerveSubsystem().getPose2d());
        // Optional<EstimatedRobotPose> cam2Estimate = cameras.getEstimatedPoseCam2();

        if (cam1Estimate.isPresent()){
            // SmartDashboard.putNumber("EstPoseX", cam1Estimate.get().estimatedPose.toPose2d().getX());
            // SmartDashboard.putNumber("EstPoseY", cam1Estimate.get().estimatedPose.toPose2d().getY());
            swerve.addVisionMeasurement(
                cam1Estimate.get().estimatedPose.toPose2d(), 
                cam1Estimate.get().timestampSeconds
                ); //VecBuilder.fill(0.5,0.5,0.5) 
        }

        // if (cam2Estimate.isPresent()){
        //     swerve.addVisionMeasurement(
        //         cam2Estimate.get().estimatedPose0.toPose2d(), 
        //         cam2Estimate.get().timestampSeconds);
        // }
    }

    @Override
    public void robotInit() {
        //FollowPathCommand.warmupCommand().schedule(); // prevent delay when running autos
        /* 
        audioConfigs.withAllowMusicDurDisable(true);
        orchestra.addInstrument(fx1);
        orchestra.addInstrument(fx2);
        orchestra.addInstrument(fx3);
        orchestra.addInstrument(fx4);
        orchestra.loadMusic("output.chrp");
        orchestra.play();
        */
    }

    @Override
    public void disabledInit() {
        pivot.setSetPoint(0);
    }

    @Override
    public void disabledPeriodic() {
    }

    @Override
    public void disabledExit() {
    }

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void autonomousExit() {
    }

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void teleopExit() {
    }

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
    }

    @Override
    public void testExit() {
    }

    @Override
    public void simulationPeriodic() {

    }
}
