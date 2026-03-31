package frc.robot.subsystems.conveyor;

import org.littletonrobotics.junction.AutoLog;

public interface ConveyorIO {
    @AutoLog
    public static class ConveyorIOInputs{
        public double conveyorPositionRot;
        public double conveyorVelocityRotPerSec;
        public double conveyorCurrentAmps;
    }

    default public void updateInputs(ConveyorIOInputs io){}
}
