// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Conveyor extends SubsystemBase {
  /** Creates a new Conveyor. */

  private final String inputsName;
  private final ConveyorIO io;
  protected final ConveyorIOInputsAutoLogged inputs = new ConveyorIOInputsAutoLogged();

  public Conveyor(String inputsName, ConveyorIO io) {
    this.inputsName = inputsName;
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs(inputsName, inputs);
    // This method will be called once per scheduler run
  }
}
