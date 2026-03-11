package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;

public class ConveyorSubsystem extends SubsystemBase {
     private TalonFX conveyorMotor;
     private TalonFXConfiguration configs;
    public ConveyorSubsystem(int newConveyorID) {
        conveyorMotor = new TalonFX(newConveyorID);
        configs = new TalonFXConfiguration();
        configs.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Amps.of(10))
        .withSupplyCurrentLimitEnable(true));
        conveyorMotor.getConfigurator().apply(configs);
    }
    public void setConveyorSpeed(double speed){
        conveyorMotor.set(speed);
    }

    @Override
    public void periodic() {}
}