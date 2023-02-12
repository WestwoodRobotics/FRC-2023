// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.Constants.SwerveModuleConstants.*;

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
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.util.Conversions;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileWriter;
import java.io.IOException;

public class SwerveModule extends SubsystemBase {
    /**
     * Creates a new SwerveModule.
     */
    private static double[] turnEncoderOffsets;

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

    // constructor
    public SwerveModule(
            int moduleNum,
            TalonFX driveMotor,
            TalonFX turningMotor,
            CANCoder encoder,
            boolean invertDrive,
            boolean invertTurn,
            PIDController driveMotorPID,
            PIDController turnMotorPID,
            SimpleMotorFeedforward feedforward) {
        this.moduleNum = moduleNum;

        m_driveMotor = driveMotor;
        m_turningMotor = turningMotor;
        e_Encoder = encoder;

        m_driveFeedforward = feedforward;
        this.driveMotorPID = driveMotorPID;
        this.turnMotorPID = turnMotorPID;

        // reset encoders
        resetEncoders();

        this.drive_inverted = invertDrive;
        this.turn_inverted = invertTurn;

        driveMotor.setNeutralMode(NeutralMode.Brake);
        driveMotor.clearStickyFaults();

        turningMotor.setNeutralMode(NeutralMode.Brake);
        turningMotor.clearStickyFaults();

        TalonFXConfiguration swerveAngleFXConfig = new TalonFXConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(true, 20, 30, 0.1);

        // put in constants later
        swerveAngleFXConfig.slot0.kP = .6;
        swerveAngleFXConfig.slot0.kI = 0.0;
        swerveAngleFXConfig.slot0.kD = 12;
        swerveAngleFXConfig.slot0.kF = 0.0;
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;
        swerveAngleFXConfig.initializationStrategy = SensorInitializationStrategy.BootToZero;

        m_turningMotor.configFactoryDefault();
        m_turningMotor.configAllSettings(swerveAngleFXConfig);
        m_turningMotor.setInverted(invertTurn);
        m_turningMotor.setNeutralMode(NeutralMode.Coast);
        resetToAbsolute();

        // Set I term bounds of drive PID to full motor output range.
        driveMotorPID.setIntegratorRange(-C_MAX_VOLTAGE, C_MAX_VOLTAGE);
    }

    private static void saveEncoderOffset() {

        try {
            BufferedWriter writer = new BufferedWriter(new FileWriter(C_ENCODER_OFFSETS_FILE_PATH));
            for (int i = 0; i < 4; i++) {
                writer.write(Double.toString(turnEncoderOffsets[i]));
                writer.newLine();
            }
            writer.close();
        } catch (IOException e) {
            System.out.println("\u001b[31;1mFailed to write turn encoder offsets to file.\u001b[0m");
        }
    }

    /**
     * Gets the persisted encoder offset from the previous robot session.
     *
     * @return Absolute encoder's offset (in degrees) from 0 (forward).
     */
    private double getEncoderOffset() {
        if (turnEncoderOffsets == null) {
            turnEncoderOffsets = new double[4];
            try {
                BufferedReader reader = new BufferedReader(new java.io.FileReader(C_ENCODER_OFFSETS_FILE_PATH));
                for (int i = 0; i < 4; i++) {
                    turnEncoderOffsets[i] = Double.parseDouble(reader.readLine());
                }
                reader.close();
            } catch (IOException e) {
                System.out.println(
                        "\u001b[31;1mFailed to read turn encoder offsets from file, please align wheels manually, "
                                + "then reset encoders.\u001b[0m");

                for (int i = 0; i < 4; i++) {
                    turnEncoderOffsets[i] = 0;
                }
            }
        }

        return turnEncoderOffsets[moduleNum];
    }

    /**
     * Sets the persisted encoder offset.
     */
    public void setEncoderOffset() {
        if (turnEncoderOffsets == null) {
            getEncoderOffset();
        }

        double currentAngle = m_turningMotor.getSelectedSensorPosition()
                / Conversions.degreesToFalcon(1, Constants.SwerveModuleConstants.C_TURNING_MOTOR_GEAR_RATIO);
        double offset = e_Encoder.getAbsolutePosition() - currentAngle;
        turnEncoderOffsets[moduleNum] = offset;

        // TODO: Make this not write to a file every time.
        saveEncoderOffset();
    }

    private void resetToAbsolute() {
        double offset = getEncoderOffset();

        double currentAngle = (e_Encoder.getAbsolutePosition() + 360 - offset) % 360;
        double absolutePosition =
                Conversions.degreesToFalcon(currentAngle, Constants.SwerveModuleConstants.C_TURNING_MOTOR_GEAR_RATIO);

        m_turningMotor.setSelectedSensorPosition(absolutePosition);
        //        lastAngle = Math.toRadians(currentAngle);
    }

    public void resetEncoderOffset() {
        for (int i = 0; i < 4; i++) {
            turnEncoderOffsets[i] = 0;
        }

        resetToAbsolute();
    }

    // set encoder position of both motors to 0
    public void resetEncoders() {
        m_turningMotor.setSelectedSensorPosition(0);
        m_driveMotor.setSelectedSensorPosition(0);
    }

    public double getVelocity() {
        return m_driveMotor.getSelectedSensorVelocity() * C_DRIVE_ENCODER_DISTANCE_PER_PULSE * 10;
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

        double driveFeedforward = m_driveFeedforward.calculate(targetDriveSpeed);

        m_driveMotor.set(
                ControlMode.PercentOutput, (this.drive_inverted ? -1 : 1) * (driveFeedforward + driveMotorOutput));
        m_turningMotor.set(
                ControlMode.Position,
                Conversions.degreesToFalcon(outputState.angle.getDegrees(), C_TURNING_MOTOR_GEAR_RATIO));
        // System.out.println(m_turningMotor.getClosedLoopError());
        //        lastAngle = outputState.angle.getRadians();
        // testing the correct motor output
        // System.out.println(
        // System.currentTimeMillis() + ", " +
        // outputState.speedMetersPerSecond+ ", " +
        // drive_vel + ", " +
        // driveMotorOutput);
    }

    /**
     * Turn off the drive motor.
     */
    public void zeroDriveMotor() {
        m_driveMotor.set(ControlMode.PercentOutput, 0);
    }

    public void setBrakeMode(boolean mode) { // True is brake, false is coast
        m_driveMotor.setNeutralMode(mode ? NeutralMode.Brake : NeutralMode.Coast);
        m_turningMotor.setNeutralMode(NeutralMode.Brake);
    }

    public Pose2d getPose() {
        return swerveModulePose;
    }

    public void setPose(Pose2d pose) {
        swerveModulePose = pose;
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }
    public SwerveModulePosition getPosition() {
      double position =
        Conversions.FalconToMeters(
          m_driveMotor.getSelectedSensorPosition(),
          ModuleConstants.kWheelDiameterMeters,
          ModuleConstants.kDriveMotorGearRatio);
      Rotation2d angle =
        Rotation2d.fromDegrees(
          Conversions.falconToDegrees(
            m_turningMotor.getSelectedSensorPosition(),
            ModuleConstants.kSteerMotorGearRatio));
      return new SwerveModulePosition(position, angle);
    }
}