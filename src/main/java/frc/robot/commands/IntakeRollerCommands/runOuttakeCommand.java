package frc.robot.commands.IntakeRollerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class runOuttakeCommand extends Command {
IntakeSubsystem Outtake;
PivotSubsystem pivot;
ConveyorSubsystem conveyor;
  public runOuttakeCommand(IntakeSubsystem newintakeSubsystem, PivotSubsystem pivot, ConveyorSubsystem conveyor)
  {
    Outtake = newintakeSubsystem;
    this.pivot = pivot;
    this.conveyor = conveyor;
    addRequirements(Outtake);
    addRequirements(pivot);
    addRequirements(conveyor);
  }

  
  @Override
  public void initialize() {
    pivot.setSetPoint(28);
  }

  @Override
  public void execute() {
    Outtake.setSpeed(-0.45);
    conveyor.setConveyorSpeed(-0.1);
  }

  @Override
  public void end(boolean interrupted) {
     Outtake.setSpeed(0);
     conveyor.setConveyorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
