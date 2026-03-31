package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
    @AutoLog
    public static class RollerSystemIOInputs{
        public double rollerPositionRot;
        public double rollerVelocityRotPerSec;
        public double rollerCurrentAmps;
    }

    public enum RollerSystemIOMode{
        BRAKE,
        COAST,
        VOLTAGE_CONTROL
    }

    public static class RollerSystemIOOutputs{
        public RollerSystemIOMode mode = RollerSystemIOMode.BRAKE;
        public double appliedVoltage = 0.0;
    }

    default public void updateInputs(RollerSystemIOInputs io){}
    default public void updateOutputs(RollerSystemIOOutputs outputs){}

    default public void setRollerVoltage(double voltage){}
}
