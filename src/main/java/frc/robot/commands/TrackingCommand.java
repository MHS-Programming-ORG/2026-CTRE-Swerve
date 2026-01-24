
package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArduCam;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.controller.PIDController;

public class TrackingCommand extends Command {
  CommandSwerveDrivetrain swerveSubsystem;
  ArduCam camera;
  SwerveRequest.FieldCentric drive;
  PIDController drivePID;
  double x, y, z;
  public TrackingCommand(CommandSwerveDrivetrain swerveSubsystem, SwerveRequest.FieldCentric drive, ArduCam camera, double x, double y, double z) {
    drivePID = new PIDController(0.1, 0, 0);
    this.drive = drive;
    this.swerveSubsystem = swerveSubsystem;
    this.camera = camera;
    addRequirements(swerveSubsystem, camera);

    this.x = x;
    this.y = y;
    this.z = z;
  }

  @Override
  public void initialize() {

  }

  @Override
  public void execute() {
    // if(camera.cameraHasTargets()){
      swerveSubsystem.applyRequest(() -> drive
    .withVelocityX(x)
    .withVelocityY(y)
    .withRotationalRate(drivePID.calculate(-camera.getYaw(), 0)));
    // } else{
    //   swerveSubsystem.applyRequest(() -> drive
    // .withVelocityX(x)
    // .withVelocityY(y)
    // .withRotationalRate(z));
    // }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return false;
  }
}
