package frc.robot.subsystems;
import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;

public class ConveyorSubsystem extends SubsystemBase {
     private TalonFX conveyorMotor;
     private TalonFX conveyorFollower;
     private TalonFXConfiguration configs;
     private TalonFX indexer;
     
    public ConveyorSubsystem(int newConveyorID, int newIndexerID) {
        conveyorMotor = new TalonFX(newConveyorID);
        indexer = new TalonFX(newIndexerID);
        configs = new TalonFXConfiguration();
        configs.withCurrentLimits(new CurrentLimitsConfigs()
        .withSupplyCurrentLimit(Amps.of(20))
        .withSupplyCurrentLimitEnable(true));
        configs.withMotorOutput(new MotorOutputConfigs().withInverted(InvertedValue.Clockwise_Positive));
        conveyorMotor.getConfigurator().apply(configs);

        indexer.setControl(new Follower(newConveyorID, MotorAlignmentValue.Opposed));
    }
    public void setConveyorSpeed(double speed){
        conveyorMotor.set(speed);
    }

    public boolean isRunning(){
        return conveyorMotor.get() > 0;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Conveyor Current Amps", conveyorMotor.getSupplyCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Indexer Current Amps", indexer.getSupplyCurrent().getValueAsDouble());
    }
}