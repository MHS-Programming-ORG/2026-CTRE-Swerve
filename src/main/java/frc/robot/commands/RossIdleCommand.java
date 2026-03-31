package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

//
//  This command is meant to be called as a default command
//
public class RossIdleCommand extends Command {
  private ShooterSubsystem shooterSub;
  private Timer timer;
  private double idleRPS;
  private double idleDelay;

  public RossIdleCommand(ShooterSubsystem shooterSub, double idleRPS, double idleDelay) {
    this.shooterSub = shooterSub;
    addRequirements(shooterSub);

    this.timer = new Timer();
    this.idleRPS = idleRPS;
    this.idleDelay = idleDelay;
  }

  public RossIdleCommand(ShooterSubsystem shooterSub, double idleRPS) {
    // Call the main constructor with a default of idleDelay of 2.0
    this(shooterSub, idleRPS, 2.0);
  }

  @Override
  public void initialize() {
    timer.restart(); // Reset the timer

    // Initially stop the shooter subsystem motors
    shooterSub.stopShooterMotors();
    shooterSub.stopKickerMotor();
  }

  @Override
  public void execute() {
    // This function runs in a loop so it's called over and over again while the
    // command is running.
    // Check to see that some amount of time has passed since the command
    // has started.  At that point, run the shooter motors to the idle speed
    SmartDashboard.putNumber("RPS", shooterSub.getShooterShoot(idleRPS));
    if (timer.get() >= idleDelay) {
      shooterSub.setShooterVelocity(idleRPS);
    }
  }

  @Override
  public void end(boolean interrupted) {
    shooterSub.stopShooterMotors();
  }

  @Override
  public boolean isFinished() {
    return false;  // This means run the command forever until we cancel the command
  }
}
