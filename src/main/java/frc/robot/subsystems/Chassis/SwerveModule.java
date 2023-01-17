package frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.ModuleConstants;
import edu.wpi.first.math.controller.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;




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

    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed){
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);


        driveMotor = new WPI_TalonSRX(driveMotorId);
        steerMotor = new WPI_TalonSRX(steerMotorId);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoderPosition = driveMotor.getSelectedSensorPosition();
        driveEncoderVelocity = driveMotor.getSelectedSensorVelocity();
        steerEncoderPosition = steerMotor.getSelectedSensorPosition();
        steerEncoderVelocity = steerMotor.getSelectedSensorVelocity();

        driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderRot2Meter);
        driveMotor.configSelectedFeedbackCoefficient(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        steerMotor.configSelectedFeedbackCoefficient(ModuleConstants.kSteerEncoderRot2Rad);
        steerMotor.configSelectedFeedbackCoefficient(ModuleConstants.kSteerEncoderRPM2RadPerSec);
        
        drivePIDController = new PIDController(ModuleConstants.kPDrive, 0, 0);
        steerPIDController = new PIDController(ModuleConstants.kPSteer, 0, 0);
        steerPIDController.enableContinuousInput(-Math.PI, Math.PI);


        driveMotor.setSelectedSensorPosition(0); // Resets the  Drive Motor Encoder
        steerMotor.setSelectedSensorPosition(0); // Resets the Steer Motor Encoder
    }
    
}
