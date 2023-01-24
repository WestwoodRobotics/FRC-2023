package frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

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
    private final AnalogInput absoluteEncoder; // Absolute Encoder
    private final boolean absoluteEncoderReversed;// Boolean for determining if the absolute encoder is reversed
    private final double absoluteEncoderOffsetRad;// Offset for the absolute encoder

    // The constructor takes in several parameters for configuring the motors, encoders, and PID controllers
    
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        // Initialize the drive and steer motors using the provided CAN IDs
        
        driveMotor = new WPI_TalonSRX(driveMotorId);
        steerMotor = new WPI_TalonSRX(steerMotorId);

        // Set the inversion for the drive and steer motors based on the provided booleans
        
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
        
        // Initialize the drive and steer PID controllers using the constants from DriveConstants

        drivePIDController = new PIDController(DriveConstants.kPSwerveDriveDriveMotor, 0, 0);
        steerPIDController = new PIDController(DriveConstants.kPSwerveDriveSteerMotor, 0, 0);
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

        // Reset the encoder positions to zero

        driveMotor.setSelectedSensorPosition(0); // Resets the  Drive Motor Encoder
        steerMotor.setSelectedSensorPosition(0); // Resets the Steer Motor Encoder
    }
    
}
