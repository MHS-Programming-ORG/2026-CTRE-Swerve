package frc.robot.commands.IntakeRollerCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PivotSubsystem;

public class runIntakeCommand extends Command {
  PivotSubsystem pivot;
  IntakeSubsystem intake;
  public runIntakeCommand(PivotSubsystem pivot, IntakeSubsystem intake) {
    this.pivot = pivot;
    addRequirements(pivot);

    this.intake = intake;
    addRequirements(intake);
  }

  @Override
  public void initialize() {
    pivot.setSetPoint(28);
  }

  @Override
  public void execute() {
    intake.setSpeed(0.4);
  }

  @Override
  public void end(boolean interrupted) {
     intake.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
