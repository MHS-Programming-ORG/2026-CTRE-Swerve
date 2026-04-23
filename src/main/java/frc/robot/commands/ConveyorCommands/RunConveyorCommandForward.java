package frc.robot.commands.ConveyorCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ConveyorSubsystem;

public class RunConveyorCommandForward extends Command {
  ConveyorSubsystem Conveyor;
  
  public RunConveyorCommandForward(ConveyorSubsystem newConveyorCommand) {
    Conveyor = newConveyorCommand;
    addRequirements(Conveyor);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Conveyor.setConveyorSpeed(0.5);
  }

  @Override
  public void end(boolean interrupted) {
    Conveyor.setConveyorSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
