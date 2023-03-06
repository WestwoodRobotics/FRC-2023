package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;

// This class contains constants used throughout the robot code
public final class Constants {

  /**
   * controller constants
   **/

  // This class contains constants for the swerve modules
  public static final class ModuleConstants {

    // Drivetrain Dimensions
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    // Gear ratio of the drive motor
    public static final double kDriveMotorGearRatio = 6.75;
    // Gear ratio of the steer motor
    public static final double kSteerMotorGearRatio = (150.0 / 7.0);
    // Conversion factor from drive encoder rotations to meters
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    // Conversion factor from drive encoder RPM to meters per second
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    // Conversion factor from steer encoder rotations to radians
    public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
    // Conversion factor from steer encoder RPM to radians per second
    public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
    // Rated Voltage of Falcon 500's
    public static final double kFalcon500Voltage = 12;

  }

  // This class contains constants for the swerve drive system
  public static final class DriveConstants {
    public static final double maxSpeed = 3; // meters per second, controls mapped to this by direct
    public static final double maxAngularSpeed = 3 * Math.PI;
    public static final double C_kPXVision = 0.015;
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21); // TODO: Update with actual track width
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5); // TODO: Update with actual wheel base

    //Coords of frontLeftSwerveModule (Robot Relative)
    public static final double frontLeftModuleX = (-kTrackWidth / 2);
    public static final double frontLeftModuleY = (kWheelBase / 2);

    //Coords of frontRightSwerveModule (Robot Relative)
    public static final double frontRightModuleX = (kTrackWidth / 2);
    public static final double frontRightModuleY = (kWheelBase / 2);

    //Coords of backLeftSwerveModule (Robot Relative)
    public static final double backLeftModuleX = (-kTrackWidth / 2);
    public static final double backLeftModuleY = (-kWheelBase / 2);

    //Coords of backRightSwerveModule (Robot Relative)
    public static final double backRightModuleX = (kTrackWidth / 2);
    public static final double backRightModuleY = (-kWheelBase / 2);

    // Kinematics of the swerve drive system
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(frontLeftModuleX, frontLeftModuleY),
      new Translation2d(frontRightModuleX, frontRightModuleY),
      new Translation2d(backLeftModuleX, backLeftModuleY),
      new Translation2d(backRightModuleX, backRightModuleY));
  }

  public static final class AutoConstants {

    public static final double kMaxVel = Units.inchesToMeters(20);
    public static final double kMaxAccel = Units.inchesToMeters(20);
    // Maximum speed of the robot in meters per second
    public static final double kMaxSpeedMetersPerSecond = kMaxVel / 4;
    // Maximum angular speed of the robot in radians per second
    public static final double kMaxAngularSpeedRadiansPerSecond = kMaxAccel / 10;
    // Maximum linear acceleration of the robot in meters per second squared
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    // Maximum angular acceleration of the robot in radians per second squared
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
    // max angular acceleration
    // Proportional gain for the X controller
    public static final double kPXController = 1.5;
    // Proportional gain for the Y controller
    public static final double kPYController = 1.5;
    // Proportional gain for the Theta controller
    public static final double kPThetaController = 3;

    // Constraints for the Theta controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    // Deadband for the driver controller axes
    public static final double kDeadzoneCircle = 0.08;
    public static final double kDeadzoneRectangle = 0.04;
  }

  public static final class IntakeConstants {
    public static final int CANID_INTAKE = 23;
  }

  public static final class TransportConstants {
    public static final int CANID_SHOULDER_LEAD = 20;
    public static final int CANID_SHOULDER_FOLLOW_1 = 21;
    public static final int CANID_SHOULDER_FOLLOW_2 = 22;
    public static final int CANID_ELBOW = 29;
    public static final int CANID_WRIST = 34;

    public static final double MAX_SHOULDER_TICKS = 360000;
    public static final double MAX_ELBOW_TICKS = 420000;
    public static final double MIN_SHOULDER_TICKS = 0;
    public static final double MIN_ELBOW_TICKS = 0;

    public static final double HIGH_SHOULDER_TICKS = 247000;
    public static final double HIGH_ELBOW_TICKS = 160000;
    public static final double MID_SHOULDER_TICKS = 272000;
    public static final double MID_ELBOW_TICKS = 186000;
    public static final double START_SHOULDER_TICKS = 0;
    public static final double START_ELBOW_TICKS = 0;
    public static final double GROUND_SHOULDER_TICKS = 0;
    public static final double GROUND_ELBOW_TICKS = 125000;
    public static final double VERTICAL_SHOULDER_TICKS = 170000;
    public static final double VERTICAL_ELBOW_TICKS = 0;
    public static final double SHELF_SHOULDER_TICKS = 66000;
    public static final double SHELF_ELBOW_TICKS = 193000;
    //public static final int FALCON500_OGTICKS = 2048;
    //public static final int FALCON500_544TICKS = 491520;
  }

  public static final class PortConstants {

    // CAN Coder Ports
    public static final int kFrontLeftCANCoderPort = 1;
    public static final int kFrontRightCANCoderPort = 3;
    public static final int kBackLeftCANCoderPort = 4;
    public static final int kBackRightCANCoderPort = 2;

    // PWM ports for the drive motors
    public static final int kFrontLeftDriveMotorPort = 17;
    public static final int kBackLeftDriveMotorPort = 13;
    public static final int kFrontRightDriveMotorPort = 16;
    public static final int kBackRightDriveMotorPort = 12;

    // PWM ports for the steer motors

    /*
    No Intake Motors Yet

    public static final int kIntakeMotorPort = 0; //TODO: Update with actual PWM port
    public static final int kIntakeMotorEncoderPort = 0;//TODO: Update with actual PWM port
    */
    
    public static final int kFrontLeftSteerMotorPort = 19;
    public static final int kBackLeftSteerMotorPort = 14;
    public static final int kFrontRightSteerMotorPort = 18;
    public static final int kBackRightSteerMotorPort = 11;

    // USB port of the driver controller
    public static final int XboxController1 = 0;
    public static final int XboxController2 = 1;

    // PWM Port for Pigeon (Gyroscope)
    public static final int kPigeonPort = 5;
  }

  public static final class FilePathConstants {
    public static final String steerEncoderOffsetSavesPath = Filesystem.getOperatingDirectory().getPath()
        + "/steerEncoderOffsets.txt";

  }

  public static final class PIDConstants {
    public static final double kPSwerveDriveDriveMotor = 0.15; // TODO: Update with actual PID values
    public static final double kISwerveDriveDriveMotor = 0; // TODO: Update with actual PID values
    public static final double kDSwerveDriveDriveMotor = 0.01; // TODO: Update with actual PID values

    public static final double kPSwerveAngle = 0.19;
    public static final double kISwerveAngle = 0.0001;
    public static final double kDSwerveAngle = 0.02;

  }

  public static final class FeedForwardConstants {
    public static final double kSwerveDriveDriveMotorStaticGainConstant = 0.0355919531; // TODO: Update with actual feedforward values
    public static final double kSwerveDriveDriveMotorVelocityGainConstant = 0.00004297063293; // TODO: Update with actual feedforward values
    public static final double kSwerveDriveDriveMotorAccelerationGainConstant = 0.0000000000355919531; // TODO: Update with actual feedforward values
  }

}
