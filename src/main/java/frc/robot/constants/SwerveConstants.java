package frc.robot.constants;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * controller constants
 **/

// This class contains constants for the swerve modules
public final class SwerveConstants {

  // Drivetrain Dimensions
  public static final double wheelDiameter = Units.inchesToMeters(3);
  // Gear ratio of the drive motor
  public static final double driveMotorGearRatio = 6.75;
  // Gear ratio of the steer motor
  public static final double steerMotorGearRatio = (150.0 / 7.0);
  // Conversion factor from drive encoder rotations to meters
//  public static final double kDriveEncoderRot2Meter = driveMotorGearRatio * Math.PI * wheelDiameter;
//  // Conversion factor from drive encoder RPM to meters per second
//  public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
  // Conversion factor from steer encoder rotations to radians
//  public static final double kSteerEncoderRot2Rad = steerMotorGearRatio * 2 * Math.PI;
//  // Conversion factor from steer encoder RPM to radians per second
//  public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;

  // Rated Voltage of Falcon 500's
  public static final double falconMaxRatedVoltage = 12;

  // Kinematics of the swerve drive system
  public static final SwerveDriveKinematics swerveDriveKinematics = new SwerveDriveKinematics(new Translation2d(ChassisConstants.frontLeftModuleX, ChassisConstants.frontLeftModuleY), new Translation2d(ChassisConstants.frontRightModuleX, ChassisConstants.frontRightModuleY), new Translation2d(ChassisConstants.backLeftModuleX, ChassisConstants.backLeftModuleY), new Translation2d(ChassisConstants.backRightModuleX, ChassisConstants.backRightModuleY));

  public static final class PID {
    // Drive PID
    public static final double driveFeedS = -0.00287211; 
    public static final double driveFeedV = 0.251512; 
    public static final double driveFeedA = 0.000147139; 
    public static final double driveP = 0.0; // TODO: Do PID
    public static final double driveI = 0; // TODO: Do PID
    public static final double driveD = 0.0; // TODO: Do PID

    // Steer PID
    public static final double steerP = 0.19;
    public static final double steerI = 0.0001;
    public static final double steerD = 0.02;
  }
}
