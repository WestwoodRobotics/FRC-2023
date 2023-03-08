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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.*;
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

public class SwerveModule extends SubsystemBase {
  private static double[] turnEncoderOffsets;
  private final int moduleNum;
  private final TalonFX driveMotor;
  private final TalonFX steerMotor;
  private final CANCoder absoluteEncoder; // Absolute Encoder
  private final PIDController drivePIDController; // PID Controller for the drive motor
  private final SimpleMotorFeedforward driveMotorFeedForward; // Feedforward for the drive motor

  /**
   * @param driveMotorCANId      The CAN ID of the drive motor
   * @param steerMotorCANId      The CAN ID of the steer motor
   * @param absoluteEncoderCANId The CAN ID of the absolute encoder
   * @param moduleNum            The module number
   */
  public SwerveModule(int driveMotorCANId, int steerMotorCANId, int absoluteEncoderCANId, int moduleNum) {
    this.moduleNum = moduleNum;

    // Initialize the can devices using provided ids
    driveMotor = new WPI_TalonFX(driveMotorCANId);
    steerMotor = new WPI_TalonFX(steerMotorCANId);

    absoluteEncoder = new CANCoder(absoluteEncoderCANId);

    driveMotor.setNeutralMode(NeutralMode.Brake);
    steerMotor.setNeutralMode(NeutralMode.Coast);
    steerMotor.setInverted(false);

    driveMotor.clearStickyFaults();
    steerMotor.clearStickyFaults();

    TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

    /* Swerve Angle Motor Configurations */
    SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1);

    swerveAngleFXConfig.slot0.kP = SwerveConstants.PID.steerP;
    swerveAngleFXConfig.slot0.kI = SwerveConstants.PID.steerI;
    swerveAngleFXConfig.slot0.kD = SwerveConstants.PID.steerD;
    swerveAngleFXConfig.slot0.kF = 0.0;
    swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

    drivePIDController = new PIDController(SwerveConstants.PID.driveP,
      SwerveConstants.PID.driveI, SwerveConstants.PID.driveD);

    steerMotor.configFactoryDefault();
    steerMotor.configAllSettings(swerveAngleFXConfig);

    resetToAbsolute();

    // Setting Integrator Range (I in PID) | (Makes sure we don't go over the voltage limit)
    drivePIDController.setIntegratorRange(-SwerveConstants.falconMaxRatedVoltage, SwerveConstants.falconMaxRatedVoltage);

    this.driveMotorFeedForward = new SimpleMotorFeedforward(SwerveConstants.PID.driveFeedS, SwerveConstants.PID.driveFeedV, SwerveConstants.PID.driveFeedA);
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
        List<String> lines = Files.readAllLines(Paths.get(FilePathConstants.steerEncoderOffsetPath),
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
      new FileWriter(FilePathConstants.steerEncoderOffsetPath))) {
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

    double currentAngle = Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), SwerveConstants.steerMotorGearRatio);
    double offset = absoluteEncoder.getAbsolutePosition() - currentAngle;
    turnEncoderOffsets[moduleNum] = offset;

    saveEncoderOffset();
    resetToAbsolute();
  }

  private void resetToAbsolute() {
    double offset = getEncoderOffset();
    double currentAngle = (absoluteEncoder.getAbsolutePosition() + 360 - offset) % 360;
    double absolutePosition = Conversions.degreesToFalcon(currentAngle, SwerveConstants.steerMotorGearRatio);
    steerMotor.setSelectedSensorPosition(absolutePosition);
    System.out.printf("\u001b[35m====Current angle: %f, offset: %f\u001b%n  Steer motor angle: %f[0m%n", currentAngle, offset, Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), SwerveConstants.steerMotorGearRatio));
    System.out.println(currentAngle - Conversions.falconToDegrees(Conversions.degreesToFalcon(currentAngle, SwerveConstants.steerMotorGearRatio), SwerveConstants.steerMotorGearRatio));
  }

  public void resetEncoderOffset() {
    for (int i = 0; i < 4; i++) {
      turnEncoderOffsets[i] = 0;
    }
    resetToAbsolute();
  }

  /**
   * Get robot velocity
   * @return return velocity in meters per second
   */

  // TODO: this is WRONG
  public double getVelocity() {
    //return Conversions.falconToRadians(driveMotor.getSelectedSensorVelocity(), SwerveModuleConstants.kDriveMotorGearRatio) * Math.PI * SwerveModuleConstants.kWheelDiameterMeters * 10;
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), SwerveConstants.wheelDiameter * Math.PI, SwerveConstants.driveMotorGearRatio);
  }

  public double getVelocity(int pididx) {
    //return Conversions.falconToRadians(driveMotor.getSelectedSensorVelocity(), SwerveModuleConstants.kDriveMotorGearRatio) * Math.PI * SwerveModuleConstants.kWheelDiameterMeters * 10;
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(pididx), 1, 1);
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocity(), Rotation2d.fromDegrees(
      steerMotor.getSelectedSensorPosition() * (360.0 / (SwerveConstants.steerMotorGearRatio * 2048.0)))); // COnverts the encoder ticks to angle
  }

  /**
   * Sets the desired state of the swerve module, using the PID controllers to
   * calculate the necessary motor outputs.
   *
   * @param state The desired state.
   */

  public void setDesiredState(SwerveModuleState state) {
//    state.speedMetersPerSecond = state.speedMetersPerSecond * 204800 / 6.12;

    double currentAngle = Conversions.falconToRadians(steerMotor.getSelectedSensorPosition(),
      SwerveConstants.steerMotorGearRatio);

    SwerveModuleState outputState = CTREModuleState.optimize(state, new Rotation2d(currentAngle));

    double angleDiff = currentAngle - outputState.angle.getRadians();
//    double targetDriveSpeed = outputState.speedMetersPerSecond * ModuleConstants.kDriveEncoderRot2Meter * 10 * Math.cos(angleDiff);
    double targetDriveSpeed = outputState.speedMetersPerSecond * Math.cos(angleDiff);

    double drive_vel = getVelocity(0);
    // Output for the drive motor (in falcon ticks)
    double driveMotorOutput = drivePIDController.calculate(drive_vel, targetDriveSpeed);
    // todo: this doesn't work at all (always returns 0)
    double driveFeedforward = driveMotorFeedForward.calculate(targetDriveSpeed);
    SmartDashboard.putNumber("Target drive speed", targetDriveSpeed);
    SmartDashboard.putNumber("Actual drive Speed", drive_vel);

//    System.out.printf("[%d] Current angle: %f6, target angle: %f6\n", moduleNum, currentAngle, outputState.angle.getRadians());

//    if (driveFeedforward != 0) System.out.printf("Drive velocity: %f -- Target speed: %f%n", drive_vel, targetDriveSpeed);
   if (driveFeedforward != 0) System.out.printf("Drive feed forward: %f -- Drive motor output: %f%n", driveFeedforward, driveMotorOutput);
    driveMotor.set(ControlMode.PercentOutput, driveFeedforward + driveMotorOutput);
    steerMotor.set(ControlMode.Position,
      Conversions.degreesToFalcon(outputState.angle.getDegrees(), SwerveConstants.steerMotorGearRatio));
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
    while (driveMotor.getSelectedSensorPosition() != 0) {
      driveMotor.setSelectedSensorPosition(0);
    }
    while (steerMotor.getSelectedSensorPosition() != 0) {
      steerMotor.setSelectedSensorPosition(0);
    }
    while (absoluteEncoder.getPosition() != 0){
      absoluteEncoder.setPosition(0);
    }

    ScheduledExecutorService scheduler = Executors.newSingleThreadScheduledExecutor();
    scheduler.schedule(this::saveEncoderOffset, 100, TimeUnit.MILLISECONDS);
    scheduler.schedule(this::resetToAbsolute, 300, TimeUnit.MILLISECONDS);
  }

  public double getAngleDegrees() {
    return Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), SwerveConstants.steerMotorGearRatio);
  }

  public SwerveModulePosition getPosition() {
    double position = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
      SwerveConstants.wheelDiameter, SwerveConstants.driveMotorGearRatio);
    Rotation2d angle = Rotation2d.fromDegrees(Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
      SwerveConstants.steerMotorGearRatio));
    return new SwerveModulePosition(position, angle);
  }

  public double setPercentVoltage(double percent)
  {
    driveMotor.set(ControlMode.PercentOutput, percent);
    return driveMotor.getMotorOutputPercent();
  }

}
