package frc.robot.commands.IntakePivotCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;

public class MoveToPositionMagicCommand extends Command {
  PivotSubsystem pivotMagic;
  double setpoint;
  double tolerance;

  public MoveToPositionMagicCommand(PivotSubsystem newPivotIntakeMagic, double newSetpoint, double newTolerance) {
    setpoint = newSetpoint;
    tolerance = newTolerance;
    pivotMagic = newPivotIntakeMagic;
    addRequirements(pivotMagic);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    pivotMagic.setSetPoint(setpoint);
  }

  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putString("isFinished", "yes");
  }

  @Override
  public boolean isFinished() {
    return MathUtil.isNear(setpoint, pivotMagic.getPivotEncoder(), tolerance);
  }
}
