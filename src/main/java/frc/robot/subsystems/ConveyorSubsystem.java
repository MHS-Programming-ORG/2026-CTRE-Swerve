package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

public class ConveyorSubsystem extends SubsystemBase {
     private TalonFX conveyorMotor;
     private TalonFX conveyorFollower;
     private TalonFXConfiguration configs;
     
     
    public ConveyorSubsystem(int newConveyorID, int newFollowerID) {
        conveyorMotor = new TalonFX(newConveyorID);
        conveyorFollower = new TalonFX(newFollowerID);
        configs = new TalonFXConfiguration();
        configs.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Amps.of(10))
        .withSupplyCurrentLimitEnable(true));
        conveyorMotor.getConfigurator().apply(configs);

        conveyorFollower.setControl(new Follower(newFollowerID, MotorAlignmentValue.Opposed));
    }
    public void setConveyorSpeed(double speed){
        conveyorMotor.set(speed);
    }

    public boolean isRunning(){
        return conveyorMotor.get() > 0;
    }

    @Override
    public void periodic() {}
}