package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ConveyorSubsystem;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;

public class CloseShootCommand extends Command {
  private ShooterSubsystem shooterSub;
  private ConveyorSubsystem conveyorSub;
  private double kickerVel, conveyorVel, goalRPS;
  private Timer timer;

  public CloseShootCommand(ShooterSubsystem shooterSub, ConveyorSubsystem conveyorSub, double kickerVel, double conveyorVel, double goalRPS) {
    this.shooterSub = shooterSub;
    addRequirements(shooterSub);

    this.conveyorSub = conveyorSub;
    addRequirements(conveyorSub);

    this.timer = new Timer();
    this.kickerVel = kickerVel;
    this.conveyorVel = conveyorVel;
    this.goalRPS = goalRPS;
  }

  @Override
  public void initialize() {
    timer.restart(); // Reset the timer

    // Run the shooter to let it spin up
    // shooterSub.setShooterVelocity(shooterVel);
    shooterSub.fixedShoot(goalRPS);
     shooterSub.setKickerVelocity(kickerVel);
    // shooterSub.setKickerVelocity(-5);
  }

  @Override
  public void execute() {
    // This function runs in a loop so it's called over and over again while the
    // command is running.
    // Check to see that some amount of time has passed since the command
    // has started.  At that point, start the conveyor and kicker
    //MathUtil.isNear(shooterVel, shooterSub.getShooterVelocity(), 6)

    //FIXME replace with velocity check
    // if (timer.get() > kickerDelay) {
    //   conveyorSub.setConveyorSpeed(conveyorVel);
    // } 
    // if(timer.get() >= kickerDelay){
    //   shooterSub.setKickerVelocity(kickerVel);
    // }
    
    // Checks velocity of one shooter motor to see if it is greater than or equal to calculated rps from ShooterCalcV2
    if(MathUtil.isNear(kickerVel, shooterSub.getKickerVelocity(), 3)){
      conveyorSub.setConveyorSpeed(conveyorVel);
    }
  }

  @Override
  public void end(boolean interrupted) {
    // Stop all motors since the command has ended.
    // This should still be called if the command in cancelled
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