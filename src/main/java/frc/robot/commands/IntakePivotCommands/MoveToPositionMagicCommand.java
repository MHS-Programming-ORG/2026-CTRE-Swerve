// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//yippie
package frc.robot.commands.IntakePivotCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class MoveToPositionMagicCommand extends Command {
  PivotSubsystem pivotMagic;
  double setpoint;
  double tolerance;
  /** Creates a new MoveToPositionMagicCommand. */
  public MoveToPositionMagicCommand(PivotSubsystem newPivotIntakeMagic, double newSetpoint, double newTolerance) {
    setpoint = newSetpoint;
    tolerance = newTolerance;
    pivotMagic = newPivotIntakeMagic;
    addRequirements(pivotMagic);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivotMagic.setSetPoint(setpoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("isFinished", "yes");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return MathUtil.isNear(setpoint, pivotMagic.getPivotEncoder(), tolerance);
  }
}
