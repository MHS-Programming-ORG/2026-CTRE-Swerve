package frc.robot.subsystems.conveyor;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;

import static frc.robot.subsystems.conveyor.ConveyorConstants.*;

public class ConveyorIOTalonFX implements ConveyorIO{
    private final TalonFX conveyor = new TalonFX(conveyorCANID);
    private final StatusSignal<Angle> conveyorPositionRot = conveyor.getPosition();
    private final StatusSignal<AngularVelocity> conveyorVelocityRotPerSec = conveyor.getVelocity();
    private final StatusSignal<Current> conveyorCurrentAmps = conveyor.getSupplyCurrent();


    public ConveyorIOTalonFX(){
    }

    @Override
    public void updateInputs(ConveyorIOInputs io){
        BaseStatusSignal.refreshAll(
            conveyorPositionRot,
            conveyorVelocityRotPerSec,
            conveyorCurrentAmps
        );

        io.conveyorPositionRot = conveyorPositionRot.getValueAsDouble();
        io.conveyorVelocityRotPerSec = conveyorVelocityRotPerSec.getValueAsDouble();
        io.conveyorCurrentAmps = conveyorCurrentAmps.getValueAsDouble();
    }
}
