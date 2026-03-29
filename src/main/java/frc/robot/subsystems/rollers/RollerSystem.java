package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.Logger;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class RollerSystem extends SubsystemBase{

    private final String inputsName;
    private final RollerSystemIO io;
    protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();

    public RollerSystem(String inputsName, RollerSystemIO io){
        this.inputsName = inputsName;
        this.io = io;
    }
    
    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);
    }
}
