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


    //resetToAbsolute();

    // Setting Integrator Range (I in PID) | (Makes sure we don't go over the voltage limit)
    drivePIDController.setIntegratorRange(-SwerveConstants.falconMaxRatedVoltage, SwerveConstants.falconMaxRatedVoltage);

    this.driveMotorFeedForward = new SimpleMotorFeedforward(SwerveConstants.PID.driveFeedS, SwerveConstants.PID.driveFeedV, SwerveConstants.PID.driveFeedA);
  }

  public SwerveModule(int driveMotorCANId, int steerMotorCANId, int moduleNum) {
    this.moduleNum = moduleNum;

    // Initialize the can devices using provided ids
    driveMotor = new WPI_TalonFX(driveMotorCANId);
    steerMotor = new WPI_TalonFX(steerMotorCANId);

    absoluteEncoder = null;

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

    //resetToAbsolute();

    // Setting Integrator Range (I in PID) | (Makes sure we don't go over the voltage limit)
    drivePIDController.setIntegratorRange(-SwerveConstants.falconMaxRatedVoltage, SwerveConstants.falconMaxRatedVoltage);

    this.driveMotorFeedForward = new SimpleMotorFeedforward(SwerveConstants.PID.driveFeedS, SwerveConstants.PID.driveFeedV, SwerveConstants.PID.driveFeedA);
  }

  private void resetToAbsolute() {
    double currentAngle = (absoluteEncoder.getAbsolutePosition()) % 360;
    double absolutePosition = -Conversions.degreesToFalcon(currentAngle, SwerveConstants.steerMotorGearRatio) + 90;
    steerMotor.setSelectedSensorPosition(-absolutePosition - 90);
    System.out.printf("\u001b[35m====Current angle: %f, offset: %f\u001b%n  Steer motor angle: %f[0m%n", currentAngle, 0.0, -Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(), SwerveConstants.steerMotorGearRatio) - 90);
    System.out.println(currentAngle - Conversions.falconToDegrees(Conversions.degreesToFalcon(currentAngle, SwerveConstants.steerMotorGearRatio), SwerveConstants.steerMotorGearRatio));
  }

  /**
   * Get robot velocity
   * @return return velocity in meters per second
   */

  public double getVelocity() {
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(), SwerveConstants.wheelDiameter * Math.PI, SwerveConstants.driveMotorGearRatio);
    //return Conversions.falconToRadians(driveMotor.getSelectedSensorVelocity(), SwerveModuleConstants.kDriveMotorGearRatio) * Math.PI * SwerveModuleConstants.kWheelDiameterMeters * 10;
  }

  public double getVelocity(int pididx) {
    //return Conversions.falconToRadians(driveMotor.getSelectedSensorVelocity(), SwerveModuleConstants.kDriveMotorGearRatio) * Math.PI * SwerveModuleConstants.kWheelDiameterMeters * 10;
    return Conversions.falconToMPS(driveMotor.getSelectedSensorVelocity(pididx), SwerveConstants.wheelDiameter * Math.PI, SwerveConstants.driveMotorGearRatio);
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

    double currentAngle = getAngle().getRadians();

    SwerveModuleState outputState = CTREModuleState.optimize(state, new Rotation2d(currentAngle));
    double angleDiff = currentAngle - outputState.angle.getRadians();
//    double targetDriveSpeed = outputState.speedMetersPerSecond * ModuleConstants.kDriveEncoderRot2Meter * 10 * Math.cos(angleDiff);
    double targetDriveSpeed = outputState.speedMetersPerSecond; //* Math.cos(angleDiff);

    double drive_vel = getVelocity();
    // Output for the drive motor (in falcon ticks)
    double driveMotorOutput = drivePIDController.calculate(drive_vel, targetDriveSpeed);
    // todo: this doesn't work at all (always returns 0)
    double driveFeedforward = driveMotorFeedForward.calculate(targetDriveSpeed);

//    System.out.printf("[%d] Current angle: %f6, target angle: %f6\n", moduleNum, currentAngle, outputState.angle.getRadians());

    driveMotor.set(ControlMode.PercentOutput, driveFeedforward + driveMotorOutput);
    
    steerMotor.set(ControlMode.Position,
      Conversions.degreesToFalcon(-outputState.angle.getDegrees() - 90, SwerveConstants.steerMotorGearRatio));
  }

  /**
   * Turn off the drive motor.
   */
  public void zeroDriveMotor() {
    driveMotor.set(ControlMode.PercentOutput, 0);
  }

  public void zeroTurnMotor()
  {
    steerMotor.set(ControlMode.Position, 0);
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
    //scheduler.schedule(this::resetToAbsolute, 300, TimeUnit.MILLISECONDS);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-Conversions.falconToDegrees(steerMotor.getSelectedSensorPosition(),
    SwerveConstants.steerMotorGearRatio) - 90);
  }

  public SwerveModulePosition getPosition() {
    double position = Conversions.falconToMeters(driveMotor.getSelectedSensorPosition(),
      SwerveConstants.wheelDiameter*Math.PI, SwerveConstants.driveMotorGearRatio);
    Rotation2d angle = getAngle();
    return new SwerveModulePosition(position, angle);
  }

  public double setPercentVoltage(double percent)
  {
    driveMotor.set(ControlMode.PercentOutput, percent);
    return driveMotor.getMotorOutputPercent();
  }
  
  public double getDriveMotorEncoderValue(){
    return driveMotor.getSelectedSensorPosition();
  }

  public double getSteerMotorEncoderValue(){
    return steerMotor.getSelectedSensorPosition();
  }
}
