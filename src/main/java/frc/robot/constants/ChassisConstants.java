package frc.robot.constants;

import edu.wpi.first.math.util.Units;

public class ChassisConstants {
  // Distance between right and left wheels
  public static final double wheelSeparationWidth = Units.inchesToMeters(21);
  //Coords of backRightSwerveModule (Robot Relative)
  public static final double backRightModuleX = (-wheelSeparationWidth / 2);
  //Coords of backLeftSwerveModule (Robot Relative)
  public static final double backLeftModuleX = (wheelSeparationWidth / 2);
  //Coords of frontRightSwerveModule (Robot Relative)
  public static final double frontRightModuleX = (-wheelSeparationWidth / 2);
  //Coords of frontLeftSwerveModule (Robot Relative)
  public static final double frontLeftModuleX = (wheelSeparationWidth / 2);
  // Distance between front and back wheels
  public static final double wheelSeparationLength = Units.inchesToMeters(25.5);
  public static final double backRightModuleY = (-wheelSeparationLength / 2);
  public static final double backLeftModuleY = (-wheelSeparationLength / 2);
  public static final double frontRightModuleY = (wheelSeparationLength / 2);
  public static final double frontLeftModuleY = (wheelSeparationLength / 2);
}
