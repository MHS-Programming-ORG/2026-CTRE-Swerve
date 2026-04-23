package frc.robot.commands.IntakePivotCommands;

import org.ejml.equation.IntegerSequence.Range;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class AgitatePivotCommand extends Command {
  IntakeSubsystem intake;
  PivotSubsystem pivot;
  double pivotIn = 20;
  double pivotOut = 28;
  boolean MovingInOrOut;
  Timer timer = new Timer();

  public AgitatePivotCommand(PivotSubsystem newPivotSubsystem, IntakeSubsystem newintakeSubsystem) {
    intake = newintakeSubsystem;
    pivot = newPivotSubsystem;
    addRequirements(pivot);
    addRequirements(intake);

  }

  @Override
  public void initialize() { 
    MovingInOrOut = false;
    timer.start();
    intake.setSpeed(0.35);
  }

  @Override
  public void execute() {
    double currentPos = pivot.getPivotEncoder();

    if (MovingInOrOut && currentPos >= pivotIn + 0.2) {
        MovingInOrOut = false;
    } else if (!MovingInOrOut && currentPos <= pivotOut - 0.2) {
        MovingInOrOut = true;
    }

    if (MovingInOrOut) {
        pivot.setSetPoint(pivotOut);
    } else {
        pivot.setSetPoint(pivotIn);
    }
  }

  @Override
  public void end(boolean interrupted) {
    pivot.setSetPoint(pivotIn);
    intake.setSpeed(0);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}