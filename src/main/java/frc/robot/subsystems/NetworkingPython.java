package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class NetworkingPython extends SubsystemBase{ 

    private NetworkTable table;

    public NetworkingPython() {
        table = NetworkTableInstance.getDefault().getTable("keyboard");
    }

    public boolean outtakePressed() {
        return table.getEntry("outtake").getBoolean(false);
    }

    public boolean fastRevPressed() {
        return table.getEntry("agitate").getBoolean(false);
    }

    public boolean weWinPressed() {
        return table.getEntry("WeWin!").getBoolean(false);
    }

    // public boolean fastRevPressed(){
    //     return table.getEntry("JeffRev").getBoolean(false);
    // }
}


