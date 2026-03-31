package frc.robot.subsystems.rollers;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystemIO.RollerSystemIOMode;
import frc.robot.subsystems.rollers.RollerSystemIO.RollerSystemIOOutputs;

public class RollerSystem extends SubsystemBase{

    private final String inputsName;
    private final RollerSystemIO io;
    protected final RollerSystemIOInputsAutoLogged inputs = new RollerSystemIOInputsAutoLogged();
    private final RollerSystemIOOutputs outputs = new RollerSystemIOOutputs();

    private BooleanSupplier coastOverride = () -> false;

    public RollerSystem(String inputsName, RollerSystemIO io){
        this.inputsName = inputsName;
        this.io = io;
    }
    
    public void periodicAfterScheduler(){
        io.updateOutputs(outputs);
    }

    @Override
    public void periodic(){
        io.updateInputs(inputs);
        Logger.processInputs(inputsName, inputs);

        if(DriverStation.isDisabled()){
            outputs.mode = RollerSystemIOMode.BRAKE;
            if(coastOverride.getAsBoolean()){
                outputs.mode = RollerSystemIOMode.COAST;
            }
        }
    }

    public void runOpenLoop(double volts){
        outputs.mode = RollerSystemIOMode.VOLTAGE_CONTROL;
        outputs.appliedVoltage = volts;
    }

    public void stop(){
        outputs.mode = RollerSystemIOMode.BRAKE;
    }
}
