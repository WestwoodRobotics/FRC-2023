package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

// This class contains constants used throughout the robot code
public final class Constants {

    // This class contains constants for the swerve modules
    public static final class ModuleConstants {
        
        // Drivetrain Dimensions
        public static final double kWheelDiameterMeters = Units.inchesToMeters(3);
        // Gear ratio of the drive motor
        public static final double kDriveMotorGearRatio = 1 / 5.8462; //TODO: Update with actual Gear Ratio
        // Gear ratio of the steer motor
        public static final double kSteerMotorGearRatio = 1 / 18.0; //TODO: Update with actual Gear Ratio 
        // Conversion factor from drive encoder rotations to meters
        public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
        // Conversion factor from steer encoder rotations to radians
        public static final double kSteerEncoderRot2Rad = kSteerMotorGearRatio * 2 * Math.PI;
        // Conversion factor from drive encoder RPM to meters per second
        public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
        // Conversion factor from steer encoder RPM to radians per second
        public static final double kSteerEncoderRPM2RadPerSec = kSteerEncoderRot2Rad / 60;
        //Rated Voltage of Falcon 500's
        public static final double kFalcon500Voltage = 12;
        


    }

    // This class contains constants for the swerve drive system
    public static final class DriveConstants{
        // Distance between right and left wheels
        public static final double kTrackWidth = Units.inchesToMeters(21); //TODO: Update with actual track width
        // Distance between front and back wheels
        public static final double kWheelBase = Units.inchesToMeters(25.5); //TODO: Update with actual wheel base
        // Kinematics of the swerve drive system
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        // Physical characteristics of the drivetrain
        public static final double kPhysicalWheelbase = Units.inchesToMeters(25.5); //TODO: Update with actual wheel base
        public static final double kPhysicalTrackwidth = Units.inchesToMeters(21); //TODO: Update with actual track width
        public static final double kMaxVel = Units.inchesToMeters(120); //TODO: Update with actual max velocity
        public static final double kMaxAccel = Units.inchesToMeters(120); //TODO: Update with actual max acceleration
        public static final double kMaxCentripetalAccel = Units.inchesToMeters(120); //TODO: Update with actual max centripetal acceleration
        public static final TrapezoidProfile.Constraints kDriveVelocityConstraints = new TrapezoidProfile.Constraints(kMaxVel, kMaxAccel);

        public static final double kPSwerveDriveDriveMotor = 0.5; //TODO: Update with actual PIDF values
        public static final double kISwerveDriveDriveMotor = 0; //TODO: Update with actual PIDF values
        public static final double kDSwerveDriveDriveMotor = 0; //TODO: Update with actual PIDF values
        
        public static final double kPSwerveDriveSteerMotor = 0.5; //TODO: Update with actual PIDF values
        public static final double kISwerveDriveSteerMotor = 0; //TODO: Update with actual PIDF values
        public static final double kDSwerveDriveSteerMotor = 0;//TODO: Update with actual PIDF values

        public static final double kDistanceFromCenterWidth = Units.inchesToMeters(21) / 2; //  TODO: Update with actual distance from center
        public static final double kDistanceFromCenterLength = Units.inchesToMeters(25.5) / 2; //  TODO: Update with actual distance from center


        public final static Translation2d frontRight = new Translation2d(kDistanceFromCenterWidth, kDistanceFromCenterLength);
        public final static Translation2d frontLeft = new Translation2d(kDistanceFromCenterWidth, -kDistanceFromCenterLength);
        public final static Translation2d backRight = new Translation2d(-kDistanceFromCenterWidth, kDistanceFromCenterLength);
        public final static Translation2d backLeft = new Translation2d(-kDistanceFromCenterWidth, -kDistanceFromCenterLength);

        

        



    }

    public static final class AutoConstants {
        // Maximum speed of the robot in meters per second
        public static final double kMaxSpeedMetersPerSecond = DriveConstants.kMaxVel / 4;
        // Maximum angular speed of the robot in radians per second
        public static final double kMaxAngularSpeedRadiansPerSecond = DriveConstants.kMaxAccel / 10;
        // Maximum linear acceleration of the robot in meters per second squared
        public static final double kMaxAccelerationMetersPerSecondSquared = 3; //TODO: Update with actual max acceleration
        // Maximum angular acceleration of the robot in radians per second squared 
        public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4; //TODO: Update with actual max angular acceleration
        // Proportional gain for the X controller
        public static final double kPXController = 1.5; //TODO: Update with actual PIDF values
        // Proportional gain for the Y controller
        public static final double kPYController = 1.5; //TODO: Update with actual PIDF values
        // Proportional gain for the Theta controller
        public static final double kPThetaController = 3; //TODO: Update with actual PIDF values

        // Constraints for the Theta controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
                new TrapezoidProfile.Constraints(
                        kMaxAngularSpeedRadiansPerSecond,
                        kMaxAngularAccelerationRadiansPerSecondSquared);
    }

    public static final class OIConstants {
        

        // Axis ID for the Y-axis of the driver controller
        public static final int kDriverYAxis = 1; //TODO: Update with actual axis ID
        // Axis ID for the X-axis of the driver controller
        public static final int kDriverXAxis = 0; //TODO: Update with actual axis ID
        // Axis ID for the rotational axis of the driver controller
        public static final int kDriverRotAxis = 4; //TODO: Update with actual axis ID
        // Button ID for the field-oriented button of the driver controller
        public static final int kDriverFieldOrientedButtonIdx = 1; //TODO: Update with actual button ID

        // Deadband for the driver controller axes
        public static final double kDeadbandCircle = 0.08; //TODO: Update with actual deadband
        public static final double kDeadBandRectangle = 0.12; //TODO: Update with actual deadband
    }

    public static final class IntakeConstants {

    }

    public static final class PortConstants {
        //CAN Coder Ports
        public static final int kFrontLeftCANCoderPort = 0; // TODO: Update with actual CANCoder port
        public static final int kFrontRightCANCoderPort = 0; // TODO: Update with actual CANCoder port
        public static final int kBackLeftCANCoderPort = 0;  // TODO: Update with actual CANCoder port
        public static final int kBackRightCANCoderPort = 0; // TODO: Update with actual CANCoder port

        // PWM ports for the drive motors
        public static final int kFrontLeftDriveMotorPort = 8; //TODO: Update with actual PWM port
        public static final int kBackLeftDriveMotorPort = 2; //TODO: Update with actual PWM port
        public static final int kFrontRightDriveMotorPort = 6;//TODO: Update with actual PWM port
        public static final int kBackRightDriveMotorPort = 4;//TODO: Update with actual PWM port

        // PWM ports for the steer motors
        public static final int kFrontLeftTurningMotorPort = 7;//TODO: Update with actual PWM port
        public static final int kBackLeftTurningMotorPort = 1;//TODO: Update with actual PWM port
        public static final int kFrontRightTurningMotorPort = 5;//TODO: Update with actual PWM port
        public static final int kBackRightTurningMotorPort = 3;//TODO: Update with actual PWM port

        // PWM ports for the steer motors
        public static final int kFrontLeftSteerMotorPort = 0;//TODO: Update with actual PWM port
        public static final int kFrontRightSteerMotorPort = 0;//TODO: Update with actual PWM port
        public static final int kBackLeftSteerMotorPort = 0;//TODO: Update with actual PWM port
        public static final int kBackRightSteerMotorPort = 0;//TODO: Update with actual PWM port

        // PWM ports for the drive motors
        public static final int kFrontLeftDriveEncoderPort = 0; //TODO: Update with actual PWM port
        public static final int kFrontRightDriveEncoderPort = 0;//TODO: Update with actual PWM port
        public static final int kBackLeftDriveEncoderPort = 0;//TODO: Update with actual PWM port
        public static final int kBackRightDriveEncoderPort = 0;//TODO: Update with actual PWM port

        // PWM ports for the intake motors
        public static final int kIntakeMotorPort = 0; //TODO: Update with actual PWM port    
        public static final int kIntakeMotorEncoderPort = 0;//TODO: Update with actual PWM port

        // USB port of the driver controller
        public static final int XboxController1 = 0;//TODO: Update with actual port
        public static final int XboxController2 = 0;//TODO: Update with actual port 
        
        // Analog input ports for the absolute encoders
        public static final int kFrontLeftDriveAbsoluteEncoderPort = 0; //TODO: Update with actual analog input port
        public static final int kBackLeftDriveAbsoluteEncoderPort = 2;//TODO: Update with actual analog input port
        public static final int kFrontRightDriveAbsoluteEncoderPort = 1;//TODO: Update with actual analog input port
        public static final int kBackRightDriveAbsoluteEncoderPort = 3;//TODO: Update with actual analog input port

        //PWM Port for Pigeon (Gyroscope)
        public static final int kPigeonPort = 0;//TODO: Update with actual PWM port
    }

    public static final class FilePathConstants{
        public static final String steerEncoderOffsetSavesPath = "/src/main/java/frc/robot/sybsystems/Chassis/EncoderOffsets/SteerEncoderOffsets.txt";
    }







}