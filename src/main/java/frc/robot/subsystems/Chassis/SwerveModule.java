package frc.robot.subsystems.Chassis;

// Import statements
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// This class represents a swerve module, which consists of a drive motor and a steer motor
// It also includes encoders and PID controllers for both the drive and steer motors, as well as an absolute encoder

public class SwerveModule {
    private final WPI_TalonSRX driveMotor; // Motor for driving
    private final WPI_TalonSRX steerMotor; // Motor for steering
    private final double driveEncoderPosition; // Encoder for measuring drive motor position
    private final double driveEncoderVelocity; // Encoder for measuring drive motor velocity
    private final double steerEncoderPosition; // Encoder for measuring drive motor position
    private final double steerEncoderVelocity; // Encoder for measuring drive motor velocity
    private final PIDController drivePIDController; // PID Controller for the drive motor
    private final PIDController steerPIDController; // PID Controller for the steer motor
    private final int absoluteEncoder; // Absolute Encoder
    private final boolean absoluteEncoderReversed;// Boolean for determining if the absolute encoder is reversed
    private final double absoluteEncoderOffsetRad;// Offset for the absolute encoder
    private final CANCoder CANCoder; // CANCoder for the Drive and Steer motors

    // The constructor
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed, int CANCoderId) {
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        this.absoluteEncoder = absoluteEncoderId;
        CANCoder = new CANCoder(CANCoderId);

        // Initialize the drive and steer motors using the provided CAN IDs
        driveMotor = new WPI_TalonSRX(driveMotorId);
        steerMotor = new WPI_TalonSRX(steerMotorId);

        // Set the inversion for the drive and steer motors based on the provided
        // booleans
        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        // Get the initial position and velocity of the encoders
        driveEncoderPosition = driveMotor.getSelectedSensorPosition();
        driveEncoderVelocity = driveMotor.getSelectedSensorVelocity();
        steerEncoderPosition = steerMotor.getSelectedSensorPosition();
        steerEncoderVelocity = steerMotor.getSelectedSensorVelocity();

        // Set the encoder coefficients for the drive and steer motors
        driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steerMotor.configSelectedFeedbackCoefficient(ModuleConstants.kSteerEncoderRot2Rad);
        steerMotor.configSelectedFeedbackCoefficient(ModuleConstants.kSteerEncoderRPM2RadPerSec);

        // Initialize the drive and steer PID controllers using the constants from
        // DriveConstants
        drivePIDController = new PIDController(DriveConstants.kPSwerveDriveDriveMotor, DriveConstants.kISwerveDriveDriveMotor, DriveConstants.kDSwerveDriveDriveMotor);
        steerPIDController = new PIDController(DriveConstants.kPSwerveDriveSteerMotor, DriveConstants.kISwerveDriveSteerMotor, DriveConstants.kDSwerveDriveSteerMotor);
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset the encoder positions to zero
        driveMotor.setSelectedSensorPosition(0); // Resets the Drive Motor Encoder
        steerMotor.setSelectedSensorPosition(0); // Resets the Steer Motor Encoder
    }

    // Method for getting the drive encoder position
    public double getDriveEncoderPosition() {
        return driveEncoderPosition;
    }

    // Method for getting the drive encoder velocity
    public double getDriveEncoderVelocity() {
        return driveEncoderVelocity;
    }

    // Method for getting the steer encoder position
    public double getSteerEncoderPosition() {
        return steerEncoderPosition;
    }

    // Method for getting the steer encoder velocity
    public double getSteerEncoderVelocity() {
        return steerEncoderVelocity;
    }

    // Method for getting the absolute encoder position
    public double getAbsoluteEncoderPosition() {
        return absoluteEncoder;
    }

    // Method for checking if the absolute encoder is reversed
    public boolean isAbsoluteEncoderReversed() {
        return absoluteEncoderReversed;
    }

    // Method for getting the absolute encoder offset
    public double getAbsoluteEncoderOffset() {
        return absoluteEncoderOffsetRad;
    }

    // Method for getting the drive PID controller
    public PIDController getDrivePIDController() {
        return drivePIDController;
    }

    // Method for setting the drive PID controller
    public void setDriveMotorSpeed(double speed) {
        driveMotor.set(speed);
    }

    // Method for getting the steer PID controller
    public PIDController getSteerPIDController() {
        return steerPIDController;
    }

    //Method for returning the CANCoder
    public CANCoder getCANCoder() {
        return CANCoder;
    }


}
