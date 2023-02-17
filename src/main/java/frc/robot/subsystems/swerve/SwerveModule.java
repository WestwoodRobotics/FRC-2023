// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.*;
import frc.robot.Constants.PIDConstants;

import frc.robot.util.Conversions;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;

    public final TalonFX m_turningMotor;
    public final TalonFX m_driveMotor;
    public final CANCoder e_Encoder;
    public final PIDController driveMotorPID;
    public final PIDController turnMotorPID;
    public final SimpleMotorFeedforward m_driveFeedforward;
    private final int moduleNum;
    private final boolean drive_inverted;
    private final boolean turn_inverted;
    Pose2d swerveModulePose = new Pose2d();
    private ShuffleboardTab tab;
    private double driveMotorOutput;
    private double turningMotorOutput;
    //    private double lastAngle;

public class SwerveModule {
  private static double[] turnEncoderOffsets;
  private final int moduleNum;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder absoluteEncoder; // Absolute Encoder
  private final PIDController drivePIDController; // PID Controller for the drive motor
  private PIDController steerPIDController; // PID Controller for the steer motor
  private final SimpleMotorFeedforward driveMotorFeedForward; // Feedforward for the drive motor
  private double driveMotorOutput; // Output for the drive motor
  private Pose2d swerveModulePose = new Pose2d();
  private static final double DEGREES_TO_FALCON = Conversions.degreesToFalcon(1, ModuleConstants.kSteerMotorGearRatio);
  private boolean encoderOffsetModified = false;

        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        e_Encoder = encoder;

  /**
   * @param driveMotorCANId         The CAN ID of the drive motor
   * @param steerMotorCANId         The CAN ID of the steer motor
   * @param isDriveMotorReversed    Boolean for determining if the drive motor is
   * @param isSteerMotorReversed    Boolean for determining if the steer motor is
   * @param absoluteEncoderCANId    The CAN ID of the absolute encoder
   * @param absoluteEncoderOffset   The offset for the absolute encoder
   */
  public SwerveModule(int driveMotorCANId, int steerMotorCANId, boolean isDriveMotorReversed, boolean isSteerMotorReversed, int absoluteEncoderCANId, double absoluteEncoderOffset, int moduleNum) {
    this.moduleNum = moduleNum;

    // Initialize the drive and steer motors using the provided CAN IDs
    driveMotor = new WPI_TalonFX(driveMotorCANId);
    steerMotor = new WPI_TalonFX(steerMotorCANId);

    // Initialize the absolute encoder using the provided CAN ID
    absoluteEncoder = new CANCoder(absoluteEncoderCANId);

    resetEncoders();

    // Set the inversion for the drive and steer motors based on the provided
    // booleans
    driveMotor.setInverted(isDriveMotorReversed);
    steerMotor.setInverted(isSteerMotorReversed);

        // put in constants later
        swerveAngleFXConfig.slot0.kP = .6;
        swerveAngleFXConfig.slot0.kI = 0.0;
        swerveAngleFXConfig.slot0.kD = 12;
        swerveAngleFXConfig.slot0.kF = 0.0;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

    TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1);

    // put in constants later
    swerveAngleFXConfig.slot0.kP = PIDConstants.kPSwerveAngle;
    swerveAngleFXConfig.slot0.kI = PIDConstants.kISwerveAngle;
    swerveAngleFXConfig.slot0.kD = PIDConstants.kDSwerveAngle;
    swerveAngleFXConfig.slot0.kF = 0.0;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
    swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

//    // Initialize the drive and steer PID controllers using the constants from
//    // DriveConstants
    drivePIDController = new PIDController(PIDConstants.kPSwerveDriveDriveMotor, PIDConstants.kISwerveDriveDriveMotor, PIDConstants.kDSwerveDriveDriveMotor);
    steerPIDController = new PIDController(PIDConstants.kPSwerveDriveSteerMotor, PIDConstants.kISwerveDriveSteerMotor, PIDConstants.kDSwerveDriveSteerMotor);
//    steerPIDController = new PIDController(DriveConstants.kPSwerveDriveSteerMotor, DriveConstants.kISwerveDriveSteerMotor, DriveConstants.kDSwerveDriveSteerMotor);
//    steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    steerMotor.configFactoryDefault();
    steerMotor.configAllSettings(swerveAngleFXConfig);



    resetToAbsolute();

    // Setting Integrator Range (I in PID) | (Makes sure we don't go over the
    // voltage limit)
    drivePIDController.setIntegratorRange(-ModuleConstants.kFalcon500Voltage, ModuleConstants.kFalcon500Voltage);
    steerPIDController.setIntegratorRange(-ModuleConstants.kFalcon500Voltage, ModuleConstants.kFalcon500Voltage);

    // TODO: remove this
//    this.driveMotorFeedForward = new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter);
    this.driveMotorFeedForward = new SimpleMotorFeedforward(0.0355919531, 0.00004297063293, 0.0000000000355919531);
  }

  /**
   * Gets the persisted encoder offset from the previous robot session.
   *
   * @return Absolute encoder's offset (in degrees) from 0 (forward).
   */
  private double getEncoderOffset() {
    if (turnEncoderOffsets == null) {
      try {
          //Reads all the lines of the file, and ignores any data beyond the Standard Character Set
          List<String> lines = Files.readAllLines(Paths.get(FilePathConstants.steerEncoderOffsetSavesPath), StandardCharsets.UTF_8);
          turnEncoderOffsets = lines.stream().limit(4).mapToDouble(Double::parseDouble).toArray();
      } catch (IOException | NumberFormatException e) {
          System.out.println(
              "\u001b[31;1mFailed to read turn encoder offsets from file, please align wheels manually, then reset encoders.\u001b[0m");
          turnEncoderOffsets = new double[4];
      }
  }

    return turnEncoderOffsets[moduleNum];
  }

  private void saveEncoderOffset() {
    if (encoderOffsetModified) {
        try (BufferedWriter writer = new BufferedWriter(new FileWriter(FilePathConstants.steerEncoderOffsetSavesPath))) {
            for (double encoderOffset : turnEncoderOffsets) {
                writer.write(Double.toString(encoderOffset));
                writer.newLine();
            }
            encoderOffsetModified = false;
        } catch (IOException e) {
            System.out.println("\u001b[31;1mFailed to write turn encoder offsets to file.\u001b[0m");
        }
    }
}

  public void setEncoderOffset() {
    if (turnEncoderOffsets == null) {
      getEncoderOffset();
    }

    double currentAngle = steerMotor.getSelectedSensorPosition() / DEGREES_TO_FALCON;
    double offset = absoluteEncoder.getAbsolutePosition() - currentAngle;
    turnEncoderOffsets[moduleNum] = offset;
    encoderOffsetModified = true;

    saveEncoderOffset();
  }

  private void resetToAbsolute() {
    double offset = getEncoderOffset();

    double currentAngle = (absoluteEncoder.getAbsolutePosition() + 360 - offset) % 360;
    double absolutePosition =
      Conversions.degreesToFalcon(currentAngle, ModuleConstants.kSteerMotorGearRatio);
      steerMotor.setSelectedSensorPosition(absolutePosition);
  }

  public void resetEncoderOffset() {
    for (int i = 0; i < 4; i++) {
      turnEncoderOffsets[i] = 0;
    }

    resetToAbsolute();
  }

        double currentAngle = m_turningMotor.getSelectedSensorPosition()
                / Conversions.degreesToFalcon(1, Constants.SwerveModuleConstants.C_TURNING_MOTOR_GEAR_RATIO);
        double offset = e_Encoder.getAbsolutePosition() - currentAngle;
        turnEncoderOffsets[moduleNum] = offset;

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(steerMotor.getSelectedSensorPosition() * (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0)))); //COnverts the encoder ticks to angle
  }

  /**
   * Sets the desired state of the swerve module, using the PID controllers to calculate the necessary motor outputs.
   *
   * @param state The desired state.
   */
  public void setDesiredState(SwerveModuleState state) {
    state.speedMetersPerSecond = state.speedMetersPerSecond * 204800 / 6.12;

    double currentAngle = Conversions.FalconToRadians(steerMotor.getSelectedSensorPosition(), ModuleConstants.kSteerMotorGearRatio);

    SwerveModuleState outputState = CTREModuleState.optimize(state, new Rotation2d(currentAngle));

    public void resetEncoderOffset() {
        for (int i = 0; i < 4; i++) {
            turnEncoderOffsets[i] = 0;
        }

    System.out.printf("Trying to reach angle %f%n", outputState.angle.getDegrees());

    double drive_vel = getVelocity();
    driveMotorOutput = drivePIDController.calculate(drive_vel, targetDriveSpeed);

    double driveFeedforward = driveMotorFeedForward.calculate(targetDriveSpeed);

    driveMotor.set(ControlMode.PercentOutput, driveFeedforward + driveMotorOutput);
    steerMotor.set(ControlMode.Position, Conversions.degreesToFalcon(outputState.angle.getDegrees(), ModuleConstants.kSteerMotorGearRatio));
  }

    public SwerveModuleState getState() {
        return new SwerveModuleState(
                getVelocity(),
                Rotation2d.fromDegrees(Conversions.falconToDegrees(
                        m_turningMotor.getSelectedSensorPosition(), C_TURNING_MOTOR_GEAR_RATIO)));
    }

    /**
     * Sets the desired state of the swerve module, using the PID controllers to calculate the necessary motor outputs.
     *
     * @param state The desired state.
     */
    public void setDesiredState(SwerveModuleState state) {
        state.speedMetersPerSecond = state.speedMetersPerSecond * 204800 / 6.12;

        double currentAngle =
                Conversions.FalconToRadians(m_turningMotor.getSelectedSensorPosition(), C_TURNING_MOTOR_GEAR_RATIO);

        SwerveModuleState outputState = SwerveModuleState.optimize(state, new Rotation2d(currentAngle));

        double angleDiff = currentAngle - outputState.angle.getRadians();
        double targetDriveSpeed = outputState.speedMetersPerSecond * Math.cos(angleDiff);

        double drive_vel = getVelocity();
        driveMotorOutput = driveMotorPID.calculate(drive_vel, targetDriveSpeed);

  public SwerveModulePosition getPosition() {
    double position = Conversions.FalconToMeters(driveMotor.getSelectedSensorPosition(), ModuleConstants.kWheelDiameterMeters, ModuleConstants.kDriveMotorGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), ModuleConstants.kSteerMotorGearRatio));
    return new SwerveModulePosition(position, angle);
  }
}
