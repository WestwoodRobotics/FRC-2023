package frc.robot.subsystems.swerve;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.Constants.FeedForwardConstants;
import frc.robot.Constants.FilePathConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.PIDConstants;
import frc.robot.util.Conversions;

import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.Executors;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

// This class represents a swerve module, which consists of a drive motor and a steer motor
// It also includes encoders and PID controllers for both the drive and steer motors, as well as an absolute encoder

public class SwerveModule {
  private static double[] turnEncoderOffsets;
  private final int moduleNum;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder absoluteEncoder; // Absolute Encoder
  private final PIDController drivePIDController; // PID Controller for the drive motor
  private final SimpleMotorFeedforward driveMotorFeedForward; // Feedforward for the drive motor
  private double driveMotorOutput; // Output for the drive motor
  private Pose2d swerveModulePose = new Pose2d();

  /**
   * @param driveMotorCANId       The CAN ID of the drive motor
   * @param steerMotorCANId       The CAN ID of the steer motor
   * @param isDriveMotorReversed  Boolean for determining if the drive motor is
   * @param isSteerMotorReversed  Boolean for determining if the steer motor is
   * @param absoluteEncoderCANId  The CAN ID of the absolute encoder
   * @param absoluteEncoderOffset The offset for the absolute encoder
   */
  public SwerveModule(int driveMotorCANId, int steerMotorCANId, boolean isDriveMotorReversed,
                      boolean isSteerMotorReversed, int absoluteEncoderCANId, double absoluteEncoderOffset, int moduleNum) {
    this.moduleNum = moduleNum;

    // Initialize the drive and steer motors using the provided CAN IDs
    driveMotor = new WPI_TalonFX(driveMotorCANId);
    steerMotor = new WPI_TalonFX(steerMotorCANId);

    // Initialize the absolute encoder using the provided CAN ID
    absoluteEncoder = new CANCoder(absoluteEncoderCANId);

    // Set the inversion for the drive and steer motors based on the provided booleans

    driveMotor.setInverted(isDriveMotorReversed);
    steerMotor.setInverted(isSteerMotorReversed);

    // Set the neutral mode for the drive and steer motors to brake
    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Coast);
    driveMotor.clearStickyFaults();
    steerMotor.clearStickyFaults();

    TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1);

    // put in constants later
    swerveAngleFXConfig.slot0.kP = PIDConstants.kPSwerveAngle;
    swerveAngleFXConfig.slot0.kI = PIDConstants.kISwerveAngle;
    swerveAngleFXConfig.slot0.kD = PIDConstants.kDSwerveAngle;
    swerveAngleFXConfig.slot0.kF = 0.0;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    // Initialize the drive and steer PID controllers using the constants from
    // DriveConstants
    drivePIDController = new PIDController(PIDConstants.kPSwerveDriveDriveMotor,
      PIDConstants.kISwerveDriveDriveMotor, PIDConstants.kDSwerveDriveDriveMotor);
    // steerPIDController = new
    // PIDController(DriveConstants.kPSwerveDriveSteerMotor,
    // DriveConstants.kISwerveDriveSteerMotor,
    // DriveConstants.kDSwerveDriveSteerMotor);
    // steerPIDController.enableContinuousInput(-Math.PI, Math.PI);

    steerMotor.configFactoryDefault();
    steerMotor.configAllSettings(swerveAngleFXConfig);

    resetToAbsolute();

    // Setting Integrator Range (I in PID) | (Makes sure we don't go over the voltage limit)
    drivePIDController.setIntegratorRange(-ModuleConstants.kFalcon500Voltage, ModuleConstants.kFalcon500Voltage);

    this.driveMotorFeedForward = new SimpleMotorFeedforward(FeedForwardConstants.kSwerveDriveDriveMotorStaticGainConstant, FeedForwardConstants.kSwerveDriveDriveMotorVelocityGainConstant, FeedForwardConstants.kSwerveDriveDriveMotorAccelerationGainConstant);
  }

  /*
   * Gets the persisted encoder offset from the previous robot session.
   *
   * @return Absolute encoder's offset (in degrees) from 0 (forward).
   */

  private double getEncoderOffset() {
    if (turnEncoderOffsets == null) {
      try {
        // Reads all the lines of the file, and ignores any data beyond the Standard
        // Character Set
        List<String> lines = Files.readAllLines(Paths.get(FilePathConstants.steerEncoderOffsetSavesPath),
          StandardCharsets.UTF_8);
        turnEncoderOffsets = lines.stream().limit(4).mapToDouble(Double::parseDouble).toArray();
      } catch (IOException | NumberFormatException e) {
        System.out.println(e.toString());
        System.out.println(
          "\u001b[31;1mFailed to read turn encoder offsets from file, please align wheels manually, then reset encoders.\u001b[0m");

        turnEncoderOffsets = new double[4];
        Arrays.fill(turnEncoderOffsets, 0);
      }
    }
    return turnEncoderOffsets[moduleNum];
  }

  private void saveEncoderOffset() {
    try (BufferedWriter writer = new BufferedWriter(
      new FileWriter(FilePathConstants.steerEncoderOffsetSavesPath))) {
      for (double encoderOffset : turnEncoderOffsets) {
        writer.write(Double.toString(encoderOffset));
        writer.newLine();
      }
    } catch (IOException e) {
      System.out.println(e.toString());
      System.out.println("\u001b[31;1mFailed to write turn encoder offsets to file.\u001b[0m");
    }
  }

  public void setEncoderOffset() {
    if (turnEncoderOffsets == null) {
      getEncoderOffset();
    }

    double currentAngle = Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), ModuleConstants.kSteerMotorGearRatio);
    double offset = absoluteEncoder.getAbsolutePosition() - currentAngle;
    turnEncoderOffsets[moduleNum] = offset;

    saveEncoderOffset();
    resetToAbsolute();
  }

  private void resetToAbsolute() {
    double offset = getEncoderOffset();
    double currentAngle = (absoluteEncoder.getAbsolutePosition() + 360 - offset) % 360;
    double absolutePosition = Conversions.degreesToFalcon(currentAngle, ModuleConstants.kSteerMotorGearRatio);
    steerMotor.setSelectedSensorPosition(absolutePosition);
    System.out.printf("\u001b[35m====Current angle: %f, offset: %f\u001b%n  Steer motor angle: %f[0m%n", currentAngle, offset, Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), ModuleConstants.kSteerMotorGearRatio));
    System.out.println(currentAngle - Conversions.falconToDegrees(Conversions.degreesToFalcon(currentAngle, ModuleConstants.kSteerMotorGearRatio), ModuleConstants.kSteerMotorGearRatio));
  }

  public void resetEncoderOffset() {
    for (int i = 0; i < 4; i++) {
      turnEncoderOffsets[i] = 0;
    }
    resetToAbsolute();
  }

  public double getVelocity() {
    return driveMotor.getSelectedSensorVelocity() * ModuleConstants.kDriveEncoderRot2Meter * 10;
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(
      steerMotor.getSelectedSensorPosition() * (360.0 / (ModuleConstants.kSteerMotorGearRatio * 2048.0)))); // COnverts the encoder ticks to angle
  }

  /**
   * Sets the desired state of the swerve module, using the PID controllers to
   * calculate the necessary motor outputs.
   *
   * @param state The desired state.
   */

  public void setDesiredState(SwerveModuleState state) {
    state.speedMetersPerSecond = state.speedMetersPerSecond * 204800 / 6.12;
    double currentAngle = Conversions.falconToRadians(steerMotor.getSelectedSensorPosition(),
      ModuleConstants.kSteerMotorGearRatio);

    SwerveModuleState outputState = CTREModuleState.optimize(state, new Rotation2d(currentAngle));

    double angleDiff = currentAngle - outputState.angle.getRadians();
    double targetDriveSpeed = outputState.speedMetersPerSecond * Math.cos(angleDiff);

//        System.out.printf("[%d] Trying to reach angle %f - %f%n", moduleNum, state.angle.getDegrees(), outputState.angle.getDegrees());
//        System.out.printf("[%d] Current absolute angle: %f %n", moduleNum, absoluteEncoder.getAbsolutePosition());
//      System.out.printf("[%d] Current encoder angle: %f %n", moduleNum, Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
//        ModuleConstants.kSteerMotorGearRatio));

    double drive_vel = getVelocity();
    driveMotorOutput = drivePIDController.calculate(drive_vel, targetDriveSpeed);

    double driveFeedforward = driveMotorFeedForward.calculate(targetDriveSpeed);

    driveMotor.set(ControlMode.PercentOutput, driveFeedforward + driveMotorOutput);
    steerMotor.set(ControlMode.Position,
      Conversions.degreesToFalcon(outputState.angle.getDegrees(), ModuleConstants.kSteerMotorGearRatio));
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

  public void resetEncoders() {
    driveMotor.setSelectedSensorPosition(0);
    steerMotor.setSelectedSensorPosition(0);
    absoluteEncoder.setPosition(0);

    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    scheduler.schedule(this::saveEncoderOffset, 100, TimeUnit.MILLISECONDS);
    scheduler.schedule(this::resetToAbsolute, 300, TimeUnit.MILLISECONDS);
  }

  public double getSteerMotorEncoderTicks() {
    return steerMotor.getSelectedSensorPosition();
  }

  public Pose2d getPose() {
    return swerveModulePose;
  }

  public void setPose(Pose2d pose) {
    swerveModulePose = pose;
  }

  public SwerveModulePosition getPosition() {
    double position = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
      ModuleConstants.kWheelDiameterMeters, ModuleConstants.kDriveMotorGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
      ModuleConstants.kSteerMotorGearRatio));
    return new SwerveModulePosition(position, angle);
  }

}
