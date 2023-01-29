package frc.robot.subsystems.Chassis;


import java.io.*;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.*;
import edu.wpi.first.math.controller.*;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.CANCoder;

// This class represents a swerve module, which consists of a drive motor and a steer motor
// It also includes encoders and PID controllers for both the drive and steer motors, as well as an absolute encoder

public class SwerveModule {
    private final WPI_TalonSRX driveMotor; // Motor for driving
    private final WPI_TalonSRX steerMotor; // Motor for steering
    private double driveEncoderPosition; // Encoder for measuring drive motor position
    private double driveEncoderVelocity; // Encoder for measuring drive motor velocity
    private double steerEncoderPosition; // Encoder for measuring drive motor position
    private double steerEncoderVelocity; // Encoder for measuring drive motor velocity
    private PIDController drivePIDController; // PID Controller for the drive motor
    private PIDController steerPIDController; // PID Controller for the steer motor
    private CANCoder absoluteEncoder; // Absolute Encoder
    private boolean absoluteEncoderReversed;// Boolean for determining if the absolute encoder is reversed
    private double absoluteEncoderOffsetRad;// Offset for the absolute encoder
    private double driveMotorOutput;
    private double turningMotorOutput;

    // The constructor
    /*
     * @param driveMotorId The CAN ID of the drive motor
     * @param steerMotorId The CAN ID of the steer motor
     * @param driveMotorReversed Boolean for determining if the drive motor is reversed
     * @param steerMotorReversed Boolean for determining if the steer motor is reversed
     * @param absoluteEncoderId The CAN ID of the absolute encoder
     * @param absoluteEncoderOffset The offset for the absolute encoder
     * @param absoluteEncoderReversed Boolean for determining if the absolute encoder is reversed
     * @param ModuleNum The module number (1-4)
     */
    public SwerveModule(int driveMotorCANId, 
                        int steerMotorCANId, 
                        boolean isDriveMotorReversed, 
                        boolean isSteerMotorReversed, 
                        int absoluteEncoderCANId, 
                        double absoluteEncoderOffset, 
                        boolean absoluteEncoderReversed, 
                        int ModuleNum){


        // Initialize the drive and steer motors using the provided CAN IDs
        driveMotor = new WPI_TalonSRX(driveMotorCANId);
        steerMotor = new WPI_TalonSRX(steerMotorCANId);


        // Initialize the absolute encoder using the provided CAN ID
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANCoder(absoluteEncoderCANId);

        //Setting Integrator Range (I in PID) | (Makes sure we don't go over the voltage limit)
        drivePIDController.setIntegratorRange(-ModuleConstants.kFalcon500Voltage, ModuleConstants.kFalcon500Voltage);

        

        // Set the inversion for the drive and steer motors based on the provided booleans
        driveMotor.setInverted(isDriveMotorReversed);
        steerMotor.setInverted(isSteerMotorReversed);

        // Set the neutral mode for the drive and steer motors to brake
        driveMotor.setNeutralMode(NeutralMode.Brake);
        steerMotor.setNeutralMode(NeutralMode.Coast);
        driveMotor.clearStickyFaults();
        steerMotor.clearStickyFaults();

        // Get the initial position and velocity of the encoders
        driveEncoderPosition = driveMotor.getSelectedSensorPosition();
        steerEncoderPosition = steerMotor.getSelectedSensorPosition();
        driveEncoderVelocity = driveMotor.getSelectedSensorVelocity();
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
    
    private void saveEncoderOffset() {
        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(FilePathConstants.steerEncoderOffsetSavesPath));
            for (int i = 0; i < 4; i++) {
                writer.write(Double.toString(this.absoluteEncoderOffsetRad));
                writer.newLine();
            }
            writer.close();
        } catch (IOException e) {
            System.out.println("\u001b[31;1mFailed to write turn encoder offsets to file.\u001b[0m");
        }
    }

    private void loadEncoderOffset() {
        if (absoluteEncoderOffsetRad == 0.0) {
            System.out.println("\u001b[31;1mAbsolute encoder offset is zero. Loading from file.\u001b[0m");
            try {
                BufferedReader reader = new BufferedReader(new FileReader(FilePathConstants.steerEncoderOffsetSavesPath));
                for (int i = 0; i < 4; i++) {
                    this.absoluteEncoderOffsetRad = Double.parseDouble(reader.readLine());
                }
                reader.close();
            } catch (IOException e) {
                System.out.println("\u001b[31;1mFailed to read turn encoder offsets from file.\u001b[0m");
            }
        }
    }

    public void setEncoderOffset() {
        if (absoluteEncoderOffsetRad == 0.0) {
            loadEncoderOffset();
        }

        double currentAngle = steerMotor.getSelectedSensorPosition()/(360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0));
        double offset = absoluteEncoder.getAbsolutePosition() - currentAngle;
        steerEncoderPosition = offset;
        saveEncoderOffset();
    }




}
