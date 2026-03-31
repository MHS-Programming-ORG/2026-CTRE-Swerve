package frc.robot.subsystems.slamtake;

import org.littletonrobotics.junction.AutoLog;

public interface SlamIO {
    @AutoLog
    public static class SlamIOInputs{
        public double slamPositionRot;
        public double slamVelocityRotPerSec;
        public double slamCurrentAmps;
        public boolean slamLSPressed;

        public double slamSetpoint;
    }

    public enum SlamIOMode{
        BRAKE,
        COAST,
        CLOSED_LOOP
    }

    public static class SlamIOOutputs{
        public SlamIOMode mode = SlamIOMode.BRAKE;

        public double position;
    }

    default public void updateInputs(SlamIOInputs io){}
    default public void updateOutputs(SlamIOOutputs outputs){}

    default public void goToPosition(double setpoint){}

    default public void setDisabledCoast(){}
    default public void setZero(){}
}
