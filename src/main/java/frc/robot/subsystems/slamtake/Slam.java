// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.slamtake;

import static frc.robot.subsystems.slamtake.SlamConstants.slamMaxRot;
import static frc.robot.subsystems.slamtake.SlamConstants.slamMinRot;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.slamtake.SlamIO.SlamIOMode;
import frc.robot.subsystems.slamtake.SlamIO.SlamIOOutputs;

public class Slam {
  private final SlamIO io;
  protected final SlamIOInputsAutoLogged inputs = new SlamIOInputsAutoLogged();
  private final SlamIOOutputs outputs = new SlamIOOutputs();

  private double goalRotation = 0.0;

  /** Creates a new Slaptake. */
  public Slam(SlamIO io) {
    this.io = io;
  }

  public void periodicAfterScheduler(){
    io.updateOutputs(outputs);
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake Pivot", inputs);

    if(DriverStation.isDisabled()){
      outputs.mode = SlamIOMode.COAST;
    }
    else{
      outputs.mode = SlamIOMode.BRAKE;
    }
    
    SmartDashboard.putNumber("Slap Encoder", inputs.slamPositionRot);
    SmartDashboard.putBoolean("Slap LS", inputs.slamLSPressed);
  }

  public void runPosition(double positionRot){
    goalRotation = positionRot;
    outputs.mode = SlamIOMode.CLOSED_LOOP;
    outputs.position = MathUtil.clamp(goalRotation, slamMinRot, slamMaxRot);
  }
}
