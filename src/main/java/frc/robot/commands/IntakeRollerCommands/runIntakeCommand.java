// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeRollerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.IntakePivotCommands.MoveToPositionMagicCommand;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class runIntakeCommand extends Command {
IntakeSubsystem Intake;
PivotSubsystem Pivot;
ConveyorSubsystem conveyor;
  public runIntakeCommand(IntakeSubsystem newintakeSubsystem, PivotSubsystem newPivotSubsystem, ConveyorSubsystem newConveyorSubsystem)
  {
    Intake = newintakeSubsystem;
    Pivot = newPivotSubsystem;
    conveyor = newConveyorSubsystem;
    addRequirements(Intake);
    addRequirements(Pivot);
    addRequirements(conveyor);
  }

  
  @Override
  public void initialize() {
    Pivot.setSetPoint(22);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Intake.setSpeed(-0.7);
    conveyor.setConveyorSpeed(0.25);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     Intake.setSpeed(0);
     conveyor.setConveyorSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
