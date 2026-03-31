package frc.robot.subsystems.rollers;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.subsystems.rollers.RollerSystemConstants.*;
import static frc.robot.subsystems.util.PhoenixUtil.*;

public class RollerSystemIOTalonFX implements RollerSystemIO{
    private final TalonFX roller = new TalonFX(rollerCANID);
    private final StatusSignal<Angle> rollerPositionRot = roller.getPosition();
    private final StatusSignal<AngularVelocity> rollerVelocityRotPerSec = roller.getVelocity();
    private final StatusSignal<Current> rollerCurrentAmps = roller.getSupplyCurrent();
    
    private double volts;
    private final VoltageOut voltageReq = new VoltageOut(0.0);  

    public RollerSystemIOTalonFX(){
        var rollerConfig = new TalonFXConfiguration();
        rollerConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        rollerConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        rollerConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        rollerConfig.CurrentLimits.SupplyCurrentLimit = rollerCurrentLimit;
        tryUntilOk(5, () -> roller.getConfigurator().apply(rollerConfig, 0.5));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            rollerPositionRot,
            rollerVelocityRotPerSec,
            rollerCurrentAmps);
        ParentDevice.optimizeBusUtilizationForAll(roller);
    }

    @Override
    public void updateInputs(RollerSystemIOInputs io){
        BaseStatusSignal.refreshAll(
            rollerPositionRot,
            rollerVelocityRotPerSec,
            rollerCurrentAmps
        );

        io.rollerPositionRot = rollerPositionRot.getValueAsDouble();
        io.rollerVelocityRotPerSec = rollerVelocityRotPerSec.getValueAsDouble();
        io.rollerCurrentAmps = rollerCurrentAmps.getValueAsDouble();
    }

    @Override
    public void updateOutputs(RollerSystemIOOutputs outputs){
        if(DriverStation.isDisabled()){
            volts = 0.0;
        }
        else{
            volts = MathUtil.clamp(outputs.appliedVoltage, -12.0, 12.0);
        }

        if(outputs.mode == RollerSystemIOMode.VOLTAGE_CONTROL){
            roller.setControl(voltageReq.withOutput(volts));
        }
    }
}
