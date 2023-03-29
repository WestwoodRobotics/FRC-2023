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
    private NetworkTableEntry verticalDiff;
    private NetworkTableEntry area;

    public Vision()
    {
        networkTable = NetworkTableInstance.getDefault().getTable("limelight");
        detected = networkTable.getEntry("tv");
        horizontalDiff = networkTable.getEntry("tx");
        verticalDiff = networkTable.getEntry("ty");
        area = networkTable.getEntry("ta");
    }

    public boolean found(){
        return(detected.getDouble(0.0) == 1);
    }

    public void setLED(boolean lightsOn)
    {
      if (lightsOn)
        networkTable.getEntry("ledMode").setValue(3);
      else
        networkTable.getEntry("ledMode").setValue(1);
    }

    public double getHorizontalDiff()
    {
        return horizontalDiff.getDouble(0.0);
    }

    public double getVerticalDiff(){
        return verticalDiff.getDouble(0.0);
    }

    public double getTargetArea(){
        if (this.found()){
            return area.getDouble(0.0);
        }
        else{
            return -1;
        }
    }


    
}
