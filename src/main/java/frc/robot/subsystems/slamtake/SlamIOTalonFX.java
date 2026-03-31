package frc.robot.subsystems.slamtake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;

import static frc.robot.subsystems.slamtake.SlamConstants.*;
import static frc.robot.subsystems.util.PhoenixUtil.*;

public class SlamIOTalonFX implements SlamIO{
    private final TalonFX slam = new TalonFX(slamCANID);
    private final DigitalInput slamLS = new DigitalInput(slamLSPort);
    private final StatusSignal<Angle> slamPositionRot = slam.getPosition();
    private final StatusSignal<AngularVelocity> slamVelocityRotPerSec = slam.getVelocity();
    private final StatusSignal<Current> slamCurrentAmps = slam.getSupplyCurrent();

    private final MotionMagicVoltage motionMagicReq = new MotionMagicVoltage(0.0);

    public SlamIOTalonFX(){
        var slamConfig = new TalonFXConfiguration();
        //Software Limits
        slamConfig.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
        slamConfig.SoftwareLimitSwitch.ForwardSoftLimitThreshold = slamMaxRot;
        slamConfig.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
        slamConfig.SoftwareLimitSwitch.ReverseSoftLimitThreshold = slamMinRot;

        //Motor Output
        slamConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        slamConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        //PID & FeedForward Constants
        slamConfig.Slot0.kP = 1;
        slamConfig.Slot0.kI = 0;
        slamConfig.Slot0.kD = 0;
        slamConfig.Slot0.kS = 0.4; 

        //Current Limits
        slamConfig.CurrentLimits.SupplyCurrentLimitEnable = false;
        slamConfig.CurrentLimits.SupplyCurrentLimit = slamCurrentLimit;

        //Motion Magic
        slamConfig.MotionMagic.MotionMagicAcceleration= 70;
        slamConfig.MotionMagic.MotionMagicCruiseVelocity = 35;
        slamConfig.MotionMagic.MotionMagicExpo_kA = 0.10000000149011612;
        slamConfig.MotionMagic.MotionMagicExpo_kV = 0.11999999731779099;
        
        tryUntilOk(5, () -> slam.getConfigurator().apply(slamConfig, 0.5));

        BaseStatusSignal.setUpdateFrequencyForAll(
            50,
            slamPositionRot,
            slamVelocityRotPerSec,
            slamCurrentAmps
        );
        slam.optimizeBusUtilization();
    }

    @Override
    public void updateInputs(SlamIOInputs io){
        BaseStatusSignal.refreshAll(
            slamPositionRot,
            slamVelocityRotPerSec,
            slamCurrentAmps
        );

        io.slamPositionRot = slamPositionRot.getValueAsDouble();
        io.slamVelocityRotPerSec = slamVelocityRotPerSec.getValueAsDouble();
        io.slamCurrentAmps = slamCurrentAmps.getValueAsDouble();
        io.slamLSPressed = !slamLS.get();
    }

    @Override
    public void goToPosition(double setpoint){
        slam.setControl(motionMagicReq.withPosition(setpoint));
    }

    @Override
    public void setDisabledCoast(){
        if(!DriverStation.isDisabled()){
            slam.setNeutralMode(NeutralModeValue.Brake);
        }
        else{
            slam.setNeutralMode(NeutralModeValue.Coast);
        }
    }

    @Override
    public void setZero(){
        if(DriverStation.isDisabled()){
            if(!slamLS.get()){
                slam.setPosition(0);
            }
        }
    }
}
