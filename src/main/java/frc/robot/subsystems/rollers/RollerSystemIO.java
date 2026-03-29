package frc.robot.subsystems.rollers;

import org.littletonrobotics.junction.AutoLog;

public interface RollerSystemIO {
    @AutoLog
    public static class RollerSystemIOInputs{
        public double rollerPositionRad;
        public double rollerVelocityRadPerSec;
        public double rollerCurrentAmps;
    }

    default void updateInputs(RollerSystemIOInputs io){}

    default void setSpeed(double speed){}
}
