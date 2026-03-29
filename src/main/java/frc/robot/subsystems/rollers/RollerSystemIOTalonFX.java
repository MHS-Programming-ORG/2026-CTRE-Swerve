package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;

import static frc.robot.subsystems.rollers.RollerSystemConstants.*;
import static frc.robot.subsystems.util.PhoenixUtil.*;

public class RollerSystemIOTalonFX implements RollerSystemIO{
    private final TalonFX rollers = new TalonFX(rollerCANID);
    private final StatusSignal<Angle> rollersPositionRot = rollers.

    public RollerSystemIOTalonFX(){
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = rollerCurrentLimit;
        tryUntilOk(5, () -> rollers.getConfigurator().apply(rollerConfig, 0.5));

        BaseStatusSignal.setUpdateFrequencyForAll(null, null);
    }
}
