// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import org.photonvision.EstimatedRobotPose;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.ArduCams;
import frc.robot.subsystems.CommandSwerveDrivetrain;

import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.AudioConfigs;


public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
    /* 

    private final TalonFX fx1 = new TalonFX(1);
    private final TalonFX fx2 = new TalonFX(2);
    private final TalonFX fx3 = new TalonFX(3);
    private final TalonFX fx4 = new TalonFX(4);
    private final Orchestra orchestra = new Orchestra();
    public final AudioConfigs audioConfigs = new AudioConfigs();
    */


    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
        
        ArduCams cameras = m_robotContainer.getCameras();
        CommandSwerveDrivetrain swerve = m_robotContainer.getSwerveSubsystem();

        Optional<EstimatedRobotPose> cam1Estimate = cameras.getEstimatedPoseCam1();
        Optional<EstimatedRobotPose> cam2Estimate = cameras.getEstimatedPoseCam2();

        if (cam1Estimate.isPresent()){
            swerve.addVisionMeasurement(
                cam1Estimate.get().estimatedPose.toPose2d(), 
                cam1Estimate.get().timestampSeconds);
        }

        if (cam2Estimate.isPresent()){
            swerve.addVisionMeasurement(
                cam2Estimate.get().estimatedPose.toPose2d(), 
                cam2Estimate.get().timestampSeconds);
        }

        
    }

    @Override
    public void robotInit() {
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
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {}

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
