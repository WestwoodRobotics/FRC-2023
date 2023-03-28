package frc.robot.subsystems.vision;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase
{
    private final NetworkTable networkTable;
    private NetworkTableEntry detected;
    private NetworkTableEntry horizontalDiff;

    public Vision()
    {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        detected = networkTable.getEntry("tv");
        horizontalDiff = networkTable.getEntry("tx");
    }

    public boolean found(){
        return(detected.getDouble(0.0) == 1);
    }

    public double getHorizontalDiff()
    {
        return horizontalDiff.getDouble(0.0);
    }
}
