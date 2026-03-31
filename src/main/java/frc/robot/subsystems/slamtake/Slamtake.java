package frc.robot.subsystems.slamtake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.rollers.RollerSystem;
import frc.robot.subsystems.rollers.RollerSystemIO;
import frc.robot.subsystems.util.FullSubsystem;
import lombok.Getter;
import lombok.Setter;

import static frc.robot.subsystems.rollers.RollerSystemConstants.*;
import static frc.robot.subsystems.slamtake.SlamConstants.*;

public class Slamtake extends FullSubsystem {
    private final Slam slam;
    private final RollerSystem roller;

    @Getter
    @Setter
    private IntakeGoal intakeGoal = IntakeGoal.STOP;
    @Getter
    @Setter
    private SlamGoal slamGoal = SlamGoal.RETRACT;

    public Slamtake(SlamIO slamIO, RollerSystemIO rollerIO) {
        this.slam = new Slam(slamIO);
        this.roller = new RollerSystem("Intake Rollers", rollerIO);
    }

    public void periodic() {
        slam.periodic();
        roller.periodic();
        
        double rollerVolts = 0.0;
        switch (intakeGoal) {
            case INTAKE:
                rollerVolts = intakeVolts;
                break;
            case OUTTAKE:
                rollerVolts = outtakeVolts;
                break;
            case STOP:
                rollerVolts = 0.0;
                break;
        }
        roller.runOpenLoop(rollerVolts);
        switch (slamGoal) {
            case DEPLOY:
                break;

            case RETRACT:
                break;

            case IDLE:
                break;
        }
    }

    public enum IntakeGoal {
        INTAKE,
        OUTTAKE,
        STOP
    }

    public enum SlamGoal {
        DEPLOY,
        RETRACT,
        IDLE
    }

    @Override
    public void periodicAfterScheduler(){
        roller.periodicAfterScheduler();
    }
}
