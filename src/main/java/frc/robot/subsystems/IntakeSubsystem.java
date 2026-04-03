// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue; 

public class IntakeSubsystem extends SubsystemBase {
private TalonFX intakeMotor;
private TalonFXConfiguration configs;

  public IntakeSubsystem(int newleaderID) {
  intakeMotor = new TalonFX(newleaderID);
  configs = new TalonFXConfiguration();
  configs.withCurrentLimits(new CurrentLimitsConfigs()
   .withSupplyCurrentLimit(Amps.of(5))
   .withSupplyCurrentLimitEnable(false));
  configs.withMotorOutput(new MotorOutputConfigs()
    .withInverted(InvertedValue.Clockwise_Positive));
  intakeMotor.getConfigurator().apply(configs);
  intakeMotor.getConfigurator().refresh(configs);
}

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }
  
  public double getEncoder(){
  return intakeMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
   SmartDashboard.putNumber("IntakeEncoder", getEncoder());
   SmartDashboard.putNumber("Intake Current", intakeMotor.getSupplyCurrent().getValueAsDouble());
  }
}
