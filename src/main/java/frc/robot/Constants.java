package frc.robot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.util.*;

// This class contains constants used throughout the robot code
public final class Constants {
  public static final int P_LOGITECH_CONTROLLER = 0;

  public static final int P_LOGITECH_CONTROLLER2 = 1;
  public static final int P_LEFT_JOY = 1;
  public static final int P_RIGHT_JOY = 0;

  /**
   * controller constants
   **/
  public static final double C_DEADZONE_CIRCLE = 0.08; // Radius of deadzone circle

  public static final double C_DEADZONE_RECTANGLE = 0.12; // Half width of deadzone rectangle
  public static final double C_JOYSTICK_EASE_SPEED_ACCEL = 0.6;
  public static final double C_JOYSTICK_EASE_SPEED_BRAKE = 0.2;

  // This class contains constants for the swerve modules
  public static final class ModuleConstants {

    // Drivetrain Dimensions
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
    // Gear ratio of the drive motor
    public static final double kDriveMotorGearRatio = 1 / 5.8462; // TODO: Update with actual Gear Ratio
    // Gear ratio of the steer motor
    public static final double kSteerMotorGearRatio = (150 / 7);
    // Conversion factor from drive encoder rotations to meters
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    // Conversion factor from steer encoder rotations to radians
    public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
    // Conversion factor from drive encoder RPM to meters per second
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    // Conversion factor from steer encoder RPM to radians per second
    public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
    // Rated Voltage of Falcon 500's
    public static final double kFalcon500Voltage = 12;

  }

  public static final class SwerveModuleConstants {
    // CAN ports move on their own?
    /**
     * The CAN IDs of the drive motors
     **/
    public static final int P_FRONT_RIGHT_TURN = 14; // 1

    public static final int P_FRONT_RIGHT_DRIVE = 13; // 2
    public static final int P_FRONT_LEFT_TURN = 11; // 3
    public static final int P_FRONT_LEFT_DRIVE = 12; // 4
    public static final int P_REAR_LEFT_TURN = 18; // 5
    public static final int P_REAR_LEFT_DRIVE = 16; // 6
    public static final int P_REAR_RIGHT_TURN = 19; // 7
    public static final int P_REAR_RIGHT_DRIVE = 17; // 8

    /**
     * CANcoder ports
     **/
    public static final int P_FRONT_RIGHT_ENCODER = 4;

    public static final int P_FRONT_LEFT_ENCODER = 2;
    public static final int P_BACK_RIGHT_ENCODER = 1;
    public static final int P_BACK_LEFT_ENCODER = 3;

    public static final String C_ENCODER_OFFSETS_FILE_PATH = Filesystem.getOperatingDirectory().getPath()
        + "/turnEncoderOffsets.txt";

    /**
     * Chassis constants, signified in meters
     **/
    public static final double C_DISTANCE_FROM_CENTER_WIDTH = 0.288925;

    public static final double C_DISTANCE_FROM_CENTER_LENGTH = 0.339725;

    /**
     * Module constants
     **/
    public static final double C_DRIVE_MOTOR_GEAR_RATIO = 6.75;

    public static final double C_TURNING_MOTOR_GEAR_RATIO = 12.8;
    public static final double C_WHEELS_DIAMETER = 0.1016; // meters
    public static final double C_WHEELS_CIRCUMFERENCE = Math.PI * C_WHEELS_DIAMETER;
    public static final double C_MAX_VOLTAGE = 12;

    public static final int C_ENCODER_CPR = 2048;

    public static final double C_DRIVE_ENCODER_DISTANCE_PER_PULSE = (C_WHEELS_DIAMETER * Math.PI)
        / ((double) C_ENCODER_CPR * SwerveModuleConstants.C_DRIVE_MOTOR_GEAR_RATIO);
    public static final double C_kTURNING_ENCODER_DISTANCE_PER_PULSE = (2.0 * Math.PI)
        / (C_ENCODER_CPR * C_TURNING_MOTOR_GEAR_RATIO); // Assumes
    // the encoders are on a 1:1 reduction with the module shaft.

    /**
     * Motor constants
     **/
    public static final double C_MAX_MOTOR_ANGULAR_SPEED = 0.02 * 2 * Math.PI; // radians/sec

    public static final double C_MAX_MOTOR_ANGULAR_ACCELERATION = 0.02 * 2 * Math.PI; // radians/s^2
    public static final double C_EDGES_PER_REVOLUTION = 2048; // encoder edges per revolution

    /**
     * Drive PID Controllers
     */
    public static final PIDController m_rRDrivePID = new PIDController(0.0000005, 0.000000005, 0.0000002);

    public static final PIDController m_rLDrivePID = new PIDController(0.0000005, 0.000000005, 0.0000002);
    public static final PIDController m_fLDrivePID = new PIDController(0.0000007, 0.00000001, 0.0000004);
    public static final PIDController m_fRDrivePID = new PIDController(0.0000007, 0.00000001, 0.0000004);
    /**
     * Turn PID Controllers
     **/
    public static final PIDController m_rRTurnPID = new PIDController(0.225, 0.002, 0.01); // double p until

    public static final PIDController // oscillations then
    // 1/10 for d, increase
    // until no oscillations then 1/100 for i
    m_rLTurnPID = new PIDController(0.2, 0.002, 0.01);
    public static final PIDController m_fLTurnPID = new PIDController(0.2, 0.002, 0.01);
    public static final PIDController m_fRTurnPID = new PIDController(0.205, 0.002, 0.01);
    // P=0.8, I=0, D=0
    // 0.6, 0.006, 0.005

    public static final SimpleMotorFeedforward m_rRDriveFeedForward = new SimpleMotorFeedforward(0.0352094709,
        0.00004316248515, 0.00000000002113902343);
    public static final SimpleMotorFeedforward m_rLDriveFeedForward = new SimpleMotorFeedforward(0.0357376904,
        0.00004255308416, 0.00000000003524346109);
    public static final SimpleMotorFeedforward m_fLDriveFeedForward = new SimpleMotorFeedforward(0.0361192778,
        0.00004295102713, 0.00000000002950698504);
    public static final SimpleMotorFeedforward m_fRDriveFeedForward = new SimpleMotorFeedforward(0.0355919531,
        0.00004297063293, 0.0000000000355919531);

    /**
     * PID constants
     **/
    public static final double C_DRIVE_kP = 0; // 2.3

    public static final double C_DRIVE_kI = 0; // 20
    public static final double C_DRIVE_kD = 0; // 0.03
    public static final double C_TURN_kP = 3.3; // 3.3 | 3.8 * Math.PI/180
    public static final double C_TURN_kI = 9.4; // 9.4
    public static final double C_TURN_kD = 0.15; // 0.15

    // Feedfoward constants drive motor
    // tiles
    public static final double C_DRIVE_kA = 0.4; // 0.4
    public static final double C_DRIVE_kS = 0.0; // .8 old value
    public static final double C_DRIVE_kV = 0.0;

    // Feedforward constants turn motor
    // tiles
    public static final double C_TURN_kA = 0.0;
    public static final double C_TURN_kS = 0.65; // 0.65
    public static final double C_TURN_kV = 0;
  }

  // This class contains constants for the swerve drive system
  public static final class DriveConstants {
    public static final double C_MAX_SPEED = 1; // meters per second, controls mapped to this by direct
    public static final double // multiplication
    C_MAX_ANGULAR_SPEED = 1.3 * Math.PI;
    public static final double C_kPXVision = 0.015;
    // Distance between right and left wheels
    public static final double kTrackWidth = Units.inchesToMeters(21); // TODO: Update with actual track width
    // Distance between front and back wheels
    public static final double kWheelBase = Units.inchesToMeters(25.5); // TODO: Update with actual wheel base
    // Kinematics of the swerve drive system
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

    // Physical characteristics of the drivetrain
    public static final double kPhysicalWheelbase = Units.inchesToMeters(25.5); // TODO: Update with actual wheel base
    public static final double kPhysicalTrackwidth = Units.inchesToMeters(21); // TODO: Update with actual track width
    public static final double kMaxVel = Units.inchesToMeters(20); // TODO: Update with actual max velocity
    public static final double kMaxAccel = Units.inchesToMeters(20); // TODO: Update with actual max acceleration
    public static final double kMaxCentripetalAccel = Units.inchesToMeters(20); // TODO: Update with actual max
                                                                                // centripetal acceleration
    public static final TrapezoidProfile.Constraints kDriveVelocityConstraints = new TrapezoidProfile.Constraints(
        kMaxVel, kMaxAccel);

    public static final double kPSwerveDriveDriveMotor = 1; // TODO: Update with actual PIDF values
    public static final double kISwerveDriveDriveMotor = 0; // TODO: Update with actual PIDF values
    public static final double kDSwerveDriveDriveMotor = 0; // TODO: Update with actual PIDF values

    public static final double kPSwerveDriveSteerMotor = 1; // TODO: Update with actual PIDF values
    public static final double kISwerveDriveSteerMotor = 0; // TODO: Update with actual PIDF values
    public static final double kDSwerveDriveSteerMotor = 0;// TODO: Update with actual PIDF values

    public static final double kDistanceFromCenterWidth = Units.inchesToMeters(21) / 2; // TODO: Update with actual
                                                                                        // distance from center
    public static final double kDistanceFromCenterLength = Units.inchesToMeters(25.5) / 2; // TODO: Update with actual
                                                                                           // distance from center

    public final static Translation2d frontRight = new Translation2d(kDistanceFromCenterWidth,
        kDistanceFromCenterLength);
    public final static Translation2d frontLeft = new Translation2d(kDistanceFromCenterWidth,
        -kDistanceFromCenterLength);
    public final static Translation2d backRight = new Translation2d(-kDistanceFromCenterWidth,
        kDistanceFromCenterLength);
    public final static Translation2d backLeft = new Translation2d(-kDistanceFromCenterWidth,
        -kDistanceFromCenterLength);

  }

  public static final class AutoConstants {
    // Maximum speed of the robot in meters per second
    public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxVel / 4;
    // Maximum angular speed of the robot in radians per second
    public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kMaxAccel / 10;
    // Maximum linear acceleration of the robot in meters per second squared
    public static final double kMaxAccelerationMetersPerSecondSquared = 3; // TODO: Update with actual max acceleration
    // Maximum angular acceleration of the robot in radians per second squared
    public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; // TODO: Update with actual
                                                                                             // max angular acceleration
    // Proportional gain for the X controller
    public static final double kPXController = 1.5;
    // Proportional gain for the Y controller
    public static final double kPYController = 1.5;
    // Proportional gain for the Theta controller
    public static final double kPThetaController = 3;

    // Constraints for the Theta controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond,
            kMaxAngularAccelerationRadiansPerSecondSquared);
  }

  public static final class OIConstants {
    // Deadband for the driver controller axes
    public static final double kDeadzoneCircle = 0.08;
    public static final double kDeadzoneRectangle = 0.12;
  }

  public static final class IntakeConstants {

  }

  public static final class PortConstants {
    // CAN Coder Ports
    public static final int kFrontLeftCANCoderPort = 2;
    public static final int kFrontRightCANCoderPort = 4;
    public static final int kBackLeftCANCoderPort = 3;
    public static final int kBackRightCANCoderPort = 1;

    // PWM ports for the drive motors
    public static final int kFrontLeftDriveMotorPort = 12;
    public static final int kBackLeftDriveMotorPort = 16;
    public static final int kFrontRightDriveMotorPort = 13;
    public static final int kBackRightDriveMotorPort = 17;

    // PWM ports for the steer motors
    public static final int kFrontLeftSteerMotorPort = 11;
    public static final int kFrontRightSteerMotorPort = 14;
    public static final int kBackLeftSteerMotorPort = 15;
    public static final int kBackRightSteerMotorPort = 18;

    // USB port of the driver controller
    public static final int XboxController1 = 1;
    public static final int XboxController2 = 2;

    // PWM Port for Pigeon (Gyroscope)
    public static final int kPigeonPort = 0;// TODO: Update with actual PWM port
  }

  public static final class FilePathConstants {
    public static final String steerEncoderOffsetSavesPath = "/home/lvuser/SteerEncoderOffsets.txt";
  }

  public static final class PIDConstants {
    public static final double kPSwerveDriveDriveMotor = 0.055; // TODO: Update with actual PID values
    public static final double kISwerveDriveDriveMotor = 0.001; // TODO: Update with actual PID values
    public static final double kDSwerveDriveDriveMotor = 0.001; // TODO: Update with actual PID values

    public static final double kPSwerveDriveSteerMotor = 0.055; // TODO: Update with actual PID values
    public static final double kISwerveDriveSteerMotor = 0.001; // TODO: Update with actual PID values
    public static final double kDSwerveDriveSteerMotor = 0.001; // TODO: Update with actual PID values

    public static final double kPSwerveAngle = 0.055; // TODO: Update with actual PID values
    public static final double kISwerveAngle = 0.001; // TODO: Update with actual PID values
    public static final double kDSwerveAngle = 0.001; // TODO: Update with actual PID values
  }

}
