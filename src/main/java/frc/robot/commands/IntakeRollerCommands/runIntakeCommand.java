// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeRollerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakePivotCommands.MoveToPositionMagicCommand;
import frc.robot.subsystems.PivotSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runIntakeCommand extends Command {
IntakeSubsystem Intake;
PivotSubsystem Pivot;
  public runIntakeCommand(IntakeSubsystem newintakeSubsystem, PivotSubsystem newPivotSubsystem)
  {
    Intake = newintakeSubsystem;
    Pivot = newPivotSubsystem;
    addRequirements(Intake);
    addRequirements(Pivot);

  }

  
  @Override
  public void initialize() {
    Pivot.setSetPoint(21.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.setSpeed(-0.45);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     Intake.setSpeed(0);
  
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
