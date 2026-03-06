// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakePivotCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.PivotSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AgitatePivotCommand extends Command {
  IntakeSubsystem intake;
  PivotSubsystem pivot;
  double pivotIn = 10.5;
  double pivotOut = 15;
  boolean MovingInOrOut;
  Timer timer = new Timer();
  /** Creates a new AgitatorCommand. */
  public AgitatePivotCommand(PivotSubsystem newPivotSubsystem, IntakeSubsystem newintakeSubsystem) {
    intake = newintakeSubsystem;
    pivot = newPivotSubsystem;
    addRequirements(pivot);
    addRequirements(intake);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() { 
    MovingInOrOut = true;
    timer.start();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pivot.setSetPoint(10.5);
    if(timer.get() > 2) {
      pivot.setSetPoint(15);
    }
    timer.stop();
    intake.setSpeed(-0.55);
    // double currentPos = pivot.getPivotEncoder();

    // if (MovingInOrOut && currentPos >= pivotOut) {
    //     MovingInOrOut = false;
    // } else if (!MovingInOrOut && currentPos <= pivotIn) {
    //     MovingInOrOut = true;
    // }

    // if (MovingInOrOut) {
    //     pivot.setSetPoint(pivotOut);
    // } else {
    //     pivot.setSetPoint(pivotIn);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    pivot.setSetPoint(pivotIn);
    intake.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}