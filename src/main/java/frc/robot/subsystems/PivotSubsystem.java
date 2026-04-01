// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;


public class PivotSubsystem extends SubsystemBase {
  
  TalonFX pivotMotor;
  DigitalInput pivotLimitSwitch;

  MotionMagicConfigs magic;
  TalonFXConfiguration configs;
  MotionMagicVoltage request;
  SoftwareLimitSwitchConfigs limit;
  

  VoltageOut voltageReq = new VoltageOut(0);
  double voltageSet = 0;

  double setPoint;

  public PivotSubsystem(int newpivotID, int limitswitchID) {
    pivotMotor = new TalonFX(newpivotID);
    pivotLimitSwitch = new DigitalInput(limitswitchID);
    
    magic = new MotionMagicConfigs();
    configs = new TalonFXConfiguration();
    request = new MotionMagicVoltage(0);
    limit = new SoftwareLimitSwitchConfigs();
    limit.ForwardSoftLimitEnable = true;
    limit.ReverseSoftLimitEnable = true;
    limit.ForwardSoftLimitThreshold = 22;
    limit.ReverseSoftLimitThreshold = 0;
    
    setPoint = 0;

    configs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    configs.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    configs.Slot0.kP = 1;
    configs.Slot0.kI = 0;
    configs.Slot0.kD = 0;
    configs.Slot0.kS = 0.4; 
    configs.Slot0.kV = 0; //Don't use this, last time we used it we broke the pivot 
    configs.Slot0.kG = 0; //Don't use this either, there is no gravity compensation on the pivot 
    configs.Slot0.kA = 0; //Don't use this either, there is no acceleration feedforward on the pivot 
    configs.withCurrentLimits(new CurrentLimitsConfigs()
    .withSupplyCurrentLimit(Amps.of(15))
    .withSupplyCurrentLimitEnable(true));

    magic.MotionMagicAcceleration = 70;
    magic.MotionMagicCruiseVelocity = 35;
    magic.MotionMagicExpo_kA = 0.10000000149011612;
    magic.MotionMagicExpo_kV = 0.11999999731779099;

    pivotMotor.getConfigurator().apply(configs);
    pivotMotor.getConfigurator().apply(magic);
    pivotMotor.getConfigurator().apply(limit);
  }

  public void setSetPoint(double newSetPoint) {
    setPoint = newSetPoint;
  }

  public double getPivotEncoder() { 
    return pivotMotor.getPosition().getValueAsDouble();
  }

  public void setSpeed(double speed) {
    pivotMotor.set(speed);

  }

  public void stopPivotMotor(){
    pivotMotor.stopMotor();
  }

  public void resetEncoder() {
    pivotMotor.setPosition(0);
  }
  public boolean isPressed() {
    return !pivotLimitSwitch.get();
  }

  public void setVoltage(double voltage){
    voltageSet = voltage;
  }

  public void setCoast(){
    pivotMotor.setNeutralMode(NeutralModeValue.Coast);
  }

  public void setBrake(){
    pivotMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("Intake-Pivot Encoders", getPivotEncoder());
    SmartDashboard.putNumber("Intake-Pivot Setpoint", setPoint);
    SmartDashboard.putBoolean("Pivot Limit Switch", isPressed());

    pivotMotor.setControl(request.withPosition(setPoint));
    // pivotMotor.setControl(voltageReq.withOutput(voltageSet));

    if(DriverStation.isDisabled()){
      if(isPressed()){
        pivotMotor.setPosition(0);
      }
      
      pivotMotor.setNeutralMode(NeutralModeValue.Coast);
    }
    else{
      pivotMotor.setNeutralMode(NeutralModeValue.Brake);
    }
  }
}
