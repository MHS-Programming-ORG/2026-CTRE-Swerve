// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;


public class ShooterSubsystem extends SubsystemBase {
  // ADD SUPPLY CURRENT LIMIT
  /** Creates a new IntakeSubsystem. */
  // {kP, kS, kV}
  private static final double[] shooterConfigVals = {0.8, 0.2967, 0.103}; //0.8, 0.1904296875, 0.12 //0.66, 0.1904296875, 0.07
  private static final double[] kickerConfigVals = {0.5, 0.345, 0.102}; //0.5, 0.4501953125, 0.06499999761581421
  private ShooterCalcV2 shooterCalcV2;
  private ArduCams camera = new ArduCams();
  private TalonFX shooterMotor1, shooterMotor2, kickerMotor;
  private final VelocityVoltage velocityRequest = new VelocityVoltage(0);
  private final VoltageOut voltageOutRequest = new VoltageOut(0);
  private double distance;

  private SlewRateLimiter slewRateLimiter = new SlewRateLimiter(7.5);

  public ShooterSubsystem(ArduCams camera, int shooterPort1, int shooterPort2, int kickerPort) {
    shooterCalcV2 = new ShooterCalcV2();
    this.camera = camera;

    shooterMotor1 = new TalonFX(shooterPort1);
    shooterMotor2 = new TalonFX(shooterPort2);
    kickerMotor = new TalonFX(kickerPort);

    var sLimitsConfig = new CurrentLimitsConfigs();
    sLimitsConfig.StatorCurrentLimit = 30;
    sLimitsConfig.SupplyCurrentLimit = 20;
    sLimitsConfig.SupplyCurrentLimitEnable = false;
    sLimitsConfig.StatorCurrentLimitEnable = false;

    var shooterConfig = new TalonFXConfiguration();
    shooterConfig.Slot0.kP = shooterConfigVals[0];
    shooterConfig.Slot0.kS = shooterConfigVals[1];
    shooterConfig.Slot0.kV = shooterConfigVals[2];
    shooterConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;

    var kickerConfig = new TalonFXConfiguration();
    kickerConfig.Slot0.kP = kickerConfigVals[0];
    kickerConfig.Slot0.kS = kickerConfigVals[1];
    kickerConfig.Slot0.kV = kickerConfigVals[2];
    kickerConfig.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
    
    shooterMotor1.getConfigurator().apply(shooterConfig);
    shooterMotor1.getConfigurator().apply(sLimitsConfig);
    shooterMotor2.getConfigurator().apply(shooterConfig);
    shooterMotor2.getConfigurator().apply(sLimitsConfig);
    kickerMotor.getConfigurator().apply(kickerConfig);
    kickerMotor.getConfigurator().apply(sLimitsConfig);

    shooterMotor1.setNeutralMode(NeutralModeValue.Coast);
    shooterMotor2.setNeutralMode(NeutralModeValue.Coast);
    kickerMotor.setNeutralMode(NeutralModeValue.Coast);

  }

  public void setShooterNKickerIdle(double shooter, double kicker){
    shooterMotor1.setControl(velocityRequest.withVelocity(shooter).withSlot(1));
    shooterMotor2.setControl(velocityRequest.withVelocity(shooter).withSlot(1));
    kickerMotor.setControl(velocityRequest.withVelocity(kicker).withSlot(1));
    if(shooterMotor1.getVelocity().getValueAsDouble() <= (shooter/100)){
      shooterMotor1.set(shooter/100);
      shooterMotor2.set(-(shooter/100));
      kickerMotor.set(kicker/100);
    }
  }

  public void setVoltage(double voltage){
    shooterMotor1.setControl(voltageOutRequest.withOutput(voltage));
    shooterMotor2.setControl(voltageOutRequest.withOutput(-voltage));
  }

  public void setKickerVoltage(double voltage){
    kickerMotor.setControl(voltageOutRequest.withOutput(voltage));
  }

  public void setKickerVelocity(double speedRPS){
    kickerMotor.setControl(velocityRequest.withVelocity(speedRPS).withSlot(0));
  }
  
  public void setShooterVelocity(double targetRPS) {
    double goalRPS = slewRateLimiter.calculate(targetRPS);
    shooterMotor1.setControl(velocityRequest.withVelocity(goalRPS).withSlot(0));
    shooterMotor2.setControl(velocityRequest.withVelocity(-goalRPS).withSlot(0));
  }

  public void stopShooterMotors(){
    shooterMotor1.stopMotor();
    shooterMotor2.stopMotor();
  }

  public void stopKickerMotor(){
    kickerMotor.stopMotor();
  }

  public double getShooterVelocity() {
    return shooterMotor1.getVelocity().getValueAsDouble();
  }

  public double getShooterVelocity2() {
    return shooterMotor2.getVelocity().getValueAsDouble();
  }

  public double getKickerVelocity(){
    return kickerMotor.getVelocity().getValueAsDouble();
  }

  // Goal: Shoot a min dist of 6feet (1.8288m) to max dist of 12 feet (3.6576m)
  // Kicker Vel > Shooter Vel == Higher Y
  // Kicker Vel < Shooter Vel == Lower Y
  // Kicker Vel = Shooter Vel == Equal Y
  public void cameraShoot(DoubleSupplier distance){
    //if(){
      this.distance = distance.getAsDouble();
      SmartDashboard.putNumber("CoordiantePositon", this.distance);
      setShooterVelocity(shooterCalcV2.getRPSForDistance(camera.getX(this.distance)));
    //}
  }

   public void fixedShoot(double goalRPS){
    //if(){
      setShooterVelocity(goalRPS);
    //}
  }

  

  public double getShooterShoot(double xDist){
    return shooterCalcV2.getRPSForDistance((camera.getX(xDist)));
  }

  @Override
  public void periodic() {
    // add speed limit here
    
    SmartDashboard.putBoolean("Camera Visable", camera.cameraVisable());
    SmartDashboard.putNumber("[Shooter] Velocity RPS", getShooterVelocity());
    SmartDashboard.putNumber("[Shooter] Velocity RPS 2", getShooterVelocity2());
    SmartDashboard.putNumber("[S] Current ", shooterMotor1.getStatorCurrent().getValueAsDouble());
    SmartDashboard.putNumber("[Shooter] Kicker", kickerMotor.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("ArduCam", camera.getX(0.0));
    SmartDashboard.putNumber("Rotor RPS",shooterMotor1.getRotorVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Shooter Accel", shooterMotor1.getAcceleration().getValueAsDouble());
    SmartDashboard.putNumber("Mechanism RPS",shooterMotor1.getVelocity().getValueAsDouble());
    SmartDashboard.putNumber("Velocity Error", shooterMotor1.getClosedLoopError().getValueAsDouble());
    SmartDashboard.putNumber("RPS", shooterCalcV2.getRPSForDistance(camera.getX(distance)));
    SmartDashboard.putNumber("Accel", shooterMotor1.getAcceleration().getValueAsDouble());
  }
}
