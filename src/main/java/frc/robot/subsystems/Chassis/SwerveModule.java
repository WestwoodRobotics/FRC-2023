package frc.robot.subsystems.Chassis;


import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.FilePathConstants;
import frc.robot.Constants.ModuleConstants;

import java.io.*;

// This class represents a swerve module, which consists of a drive motor and a steer motor
// It also includes encoders and PID controllers for both the drive and steer motors, as well as an absolute encoder

public class SwerveModule {
  private final WPI_TalonSRX driveMotor; // Motor for driving
  private final WPI_TalonSRX steerMotor; // Motor for steering
  private final double driveEncoderPosition; // Encoder for measuring drive motor position
  private final double driveEncoderVelocity; // Encoder for measuring drive motor velocity
  private double steerEncoderPosition; // Encoder for measuring drive motor position
  private final double steerEncoderVelocity; // Encoder for measuring drive motor velocity
  private PIDController drivePIDController; // PID Controller for the drive motor
  private final PIDController steerPIDController; // PID Controller for the steer motor
  private final CANCoder absoluteEncoder; // Absolute Encoder
  private final boolean absoluteEncoderReversed; // Boolean for determining if the absolute encoder is reversed
  private double absoluteEncoderOffsetRad; // Offset for the absolute encoder
  private double driveMotorOutput; // Output for the drive motor
  private double turningMotorOutput; // Output for the turning motor
  private SimpleMotorFeedforward driveMotorFeedForward; // Feedforward for the drive motor
  private final boolean isDriveMotorReversed; // Boolean for determining if the drive motor is reversed
  private final boolean isSteerMotorReversed; // Boolean for determining if the steer motor is reversed
  private Pose2d swerveModulePose = new Pose2d();


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
                      boolean absoluteEncoderReversed
  ) {


    // Initialize the drive and steer motors using the provided CAN IDs
    driveMotor = new WPI_TalonSRX(driveMotorCANId);
    steerMotor = new WPI_TalonSRX(steerMotorCANId);


    // Initialize the absolute encoder using the provided CAN ID
    this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
    this.absoluteEncoderReversed = absoluteEncoderReversed;
    absoluteEncoder = new CANCoder(absoluteEncoderCANId);

    //Setting Integrator Range (I in PID) | (Makes sure we don't go over the voltage limit)
    drivePIDController.setIntegratorRange(-ModuleConstants.kFalcon500Voltage, ModuleConstants.kFalcon500Voltage);

    this.isDriveMotorReversed = isDriveMotorReversed;
    this.isSteerMotorReversed = isSteerMotorReversed;

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
    drivePIDController = new PIDController(DriveConstants.kPSwerveDriveDriveMotor, DriveConstants.kISwerveDriveDriveMotor, DriveConstants.kDSwerveDriveDriveMotor);
    steerPIDController = new PIDController(DriveConstants.kPSwerveDriveSteerMotor, DriveConstants.kISwerveDriveSteerMotor, DriveConstants.kDSwerveDriveSteerMotor);
    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    // Reset the encoder positions to zero
    driveMotor.setSelectedSensorPosition(0); // Resets the  Drive Motor Encoder
    steerMotor.setSelectedSensorPosition(0); // Resets the Steer Motor Encoder


  }

  //Accessor Methods for private instance variables
  public double getDriveEncoderPosition() {
    return driveEncoderPosition;
  }

  public double getSteerEncoderPosition() {
    return steerEncoderPosition;
  }

  public double getDriveEncoderVelocity() {
    return driveEncoderVelocity;
  }

  public double getSteerEncoderVelocity() {
    return steerEncoderVelocity;
  }

  public boolean getAbsoluteEncoderReversed() {
    return absoluteEncoderReversed;
  }

  public double getTurningMotorOutput() {
    return turningMotorOutput;
  }

  public boolean getIsDriveMotorReversed() {
    return isDriveMotorReversed;
  }

  public boolean getIsSteerMotorReversed() {
    return isSteerMotorReversed;
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
      getEncoderOffset(); //Loads the offset
    }

    double currentAngle = (steerMotor.getSelectedSensorPosition() / (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0))); //Converts the encoder ticks to angle
    double offset = absoluteEncoder.getAbsolutePosition() - currentAngle; //Gets the offset
    steerEncoderPosition = offset; //Sets the offset to the encoder position
    saveEncoderOffset(); //Saves the offset
  }

  private double getEncoderOffset() {
    if (absoluteEncoderOffsetRad == 0.0) {
      loadEncoderOffset();
    }
    return absoluteEncoderOffsetRad;
  }

  public void setAbsoluteEncoderAsBaseline() {
    double offset = getEncoderOffset();
    double currentAngle = (absoluteEncoder.getAbsolutePosition() + 360 - offset) % 360;
    double absolutePosition = currentAngle * (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0)); //Converts the angle to encoder ticks
    steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  public double getVelocity() {
    return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRot2Meter * 10;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(
      getVelocity(),
      Rotation2d.fromDegrees(steerMotor.getSelectedSensorPosition() * (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0)))); //Converts the encoder ticks to angle
  }

  public void setDesiredState(SwerveModuleState state) {
    state.speedMetersPerSecond = state.speedMetersPerSecond * 204800 / 6.12;

    double currentAngle = steerMotor.getSelectedSensorPosition() * (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0));

    SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(currentAngle));

    double angleDiff = currentAngle - outputState.angle.getRadians();
    double targetDriveSpeed = outputState.speedMetersPerSecond * Math.cos(angleDiff);

    double drive_vel = getVelocity();
    driveMotorOutput = drivePIDController.calculate(drive_vel, targetDriveSpeed);

    double driveFeedforward = driveMotorFeedForward.calculate(targetDriveSpeed);

    driveMotor.set(ControlMode.PercentOutput, (isDriveMotorReversed ? -1 : 1) * (driveFeedforward + driveMotorOutput));
    steerMotor.set(ControlMode.Position, outputState.angle.getDegrees() / (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0))); //Converts the angle to encoder ticks
  }

  /**
   * Turn off the drive motor.
   */
  public void zeroDriveMotor() {
    driveMotor.set(ControlMode.PercentOutput, 0);
  }

  public void setBrakeMode(boolean mode) { // True is brake, false is coast
    driveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
    steerMotor.setNeutralMode(NeutralMode.Brake);
  }

  public Pose2d getPose() {
    return swerveModulePose;
  }

  public void setPose(Pose2d pose) {
    swerveModulePose = pose;
  }

  public void periodic() {
    // This method will be called once per scheduler run
  }


}
