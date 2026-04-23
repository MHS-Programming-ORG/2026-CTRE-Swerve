package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class CalculateShootCommand extends Command {
  private ShooterSubsystem shooterSub;
  private ConveyorSubsystem conveyorSub;
  private double conveyorVel;
  private DoubleSupplier distance;
  private Timer timer;

  public CalculateShootCommand(ShooterSubsystem shooterSub, ConveyorSubsystem conveyorSub, double conveyorVel,  DoubleSupplier distance) {
    this.shooterSub = shooterSub;
    addRequirements(shooterSub);

    this.conveyorSub = conveyorSub;
    addRequirements(conveyorSub);

    this.timer = new Timer();
    this.conveyorVel = conveyorVel;
    this.distance = distance;
  }

  @Override
  public void initialize() {
    timer.restart(); // Reset the timer

    // Run the shooter to let it spin up
    shooterSub.cameraShoot(distance);
    shooterSub.calculateKicker(distance);
  }

  @Override
  public void execute() {
     if(MathUtil.isNear(shooterSub.getKickerShoot(distance.getAsDouble()), shooterSub.getKickerVelocity(), 3) && MathUtil.isNear(shooterSub.getShooterShoot(distance.getAsDouble()), shooterSub.getShooterVelocity(), 3)){
      conveyorSub.setConveyorSpeed(conveyorVel);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.stopKickerMotor();
    shooterSub.stopShooterMotors();
    conveyorSub.setConveyorSpeed(0.0);
    timer.stop();
  }

  @Override
  public boolean isFinished() {
    return false;  // This means run the command forever until we cancel the command
  }
}