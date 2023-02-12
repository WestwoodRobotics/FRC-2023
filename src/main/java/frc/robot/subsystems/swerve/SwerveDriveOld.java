// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swerve;

import static frc.robot.Constants.DriveConstants.C_MAX_SPEED;
import static frc.robot.Constants.SwerveModuleConstants.*;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;

public class SwerveDriveOld extends SubsystemBase {

    /** Creates a new SwerveDrive. */
    private final Translation2d
            m_frontRightLocation = new Translation2d(C_DISTANCE_FROM_CENTER_WIDTH, C_DISTANCE_FROM_CENTER_LENGTH),
            m_frontLeftLocation = new Translation2d(-C_DISTANCE_FROM_CENTER_WIDTH, C_DISTANCE_FROM_CENTER_LENGTH),
            m_rearLeftLocation = new Translation2d(-C_DISTANCE_FROM_CENTER_WIDTH, -C_DISTANCE_FROM_CENTER_LENGTH),
            m_rearRightLocation = new Translation2d(C_DISTANCE_FROM_CENTER_WIDTH, -C_DISTANCE_FROM_CENTER_LENGTH);

    private final TalonFX frontRightDriveMotor = new TalonFX(P_FRONT_RIGHT_DRIVE),
            frontRightTurnMotor = new TalonFX(P_FRONT_RIGHT_TURN),
            frontLeftDriveMotor = new TalonFX(P_FRONT_LEFT_DRIVE),
            frontLeftTurnMotor = new TalonFX(P_FRONT_LEFT_TURN),
            rearLeftDriveMotor = new TalonFX(P_REAR_LEFT_DRIVE),
            rearLeftTurnMotor = new TalonFX(P_REAR_LEFT_TURN),
            rearRightDriveMotor = new TalonFX(P_REAR_RIGHT_DRIVE),
            rearRightTurnMotor = new TalonFX(P_REAR_RIGHT_TURN);

    // CANCoders move counter-clockwise from the top.
    public final CANCoder frontRightEncoder = new CANCoder(P_FRONT_RIGHT_ENCODER),
            frontLeftEncoder = new CANCoder(P_FRONT_LEFT_ENCODER),
            backLeftEncoder = new CANCoder(P_BACK_LEFT_ENCODER),
            backRightEncoder = new CANCoder(P_BACK_RIGHT_ENCODER);

    // Modules arranged in coordinate grid space
    private final SwerveModule
            m_frontRight =
                    new SwerveModule(
                            0,
                            frontRightDriveMotor,
                            frontRightTurnMotor,
                            frontRightEncoder,
                            false,
                            false,
                            m_fRDrivePID,
                            m_fRTurnPID,
                            m_fRDriveFeedForward),
            m_frontLeft =
                    new SwerveModule(
                            1,
                            frontLeftDriveMotor,
                            frontLeftTurnMotor,
                            frontLeftEncoder,
                            false,
                            false,
                            m_fLDrivePID,
                            m_fLTurnPID,
                            m_fLDriveFeedForward),
            m_rearLeft =
                    new SwerveModule(
                            2,
                            rearLeftDriveMotor,
                            rearLeftTurnMotor,
                            backLeftEncoder,
                            false,
                            false,
                            m_rLDrivePID,
                            m_rLTurnPID,
                            m_rLDriveFeedForward),
            m_rearRight =
                    new SwerveModule(
                            3,
                            rearRightDriveMotor,
                            rearRightTurnMotor,
                            backRightEncoder,
                            false,
                            false,
                            m_rRDrivePID,
                            m_rRTurnPID,
                            m_rRDriveFeedForward);

    // 0private AHRS imu = new AHRS();
    private final WPI_Pigeon2 imu = new WPI_Pigeon2(5);

    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontRightLocation, m_frontLeftLocation, m_rearLeftLocation, m_rearRightLocation);
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(DriveConstants.kDriveKinematics, imu.getRotation2d(),getPositions());

    private double speedMulti = 1;

    public SwerveDriveOld() {
        imu.reset();
    }
    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[0] = m_frontLeft.getPosition();
        positions[1] = m_frontRight.getPosition();
        positions[2] = m_rearLeft.getPosition();
        positions[3] = m_rearRight.getPosition();
        return positions;
      }
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, C_MAX_SPEED);
        m_frontRight.setDesiredState(swerveModuleStates[0]);
        m_frontLeft.setDesiredState(swerveModuleStates[1]);
        m_rearLeft.setDesiredState(swerveModuleStates[2]);
        m_rearRight.setDesiredState(swerveModuleStates[3]);
    }

    // Testing isolated turning
    public void turn(int dir, double speed) {
        m_frontRight.setDesiredState(new SwerveModuleState(dir * -speed, Rotation2d.fromDegrees(45)));
        m_frontLeft.setDesiredState(new SwerveModuleState(dir * speed, Rotation2d.fromDegrees(-45)));
        m_rearRight.setDesiredState(new SwerveModuleState(dir * -speed, Rotation2d.fromDegrees(-45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(dir * speed, Rotation2d.fromDegrees(45)));
    }

    // Testing more isolated turning
    public void turn(double speed) {
        ChassisSpeeds speeds = new ChassisSpeeds(0, 0, speed);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        m_frontRight.setDesiredState(moduleStates[0]);
        m_frontLeft.setDesiredState(moduleStates[1]);
        m_rearLeft.setDesiredState(moduleStates[2]);
        m_rearRight.setDesiredState(moduleStates[3]);
    }

    // Testing isolated x and y
    public void translate(double x, double y) {
        ChassisSpeeds speeds = new ChassisSpeeds(x, y, 0);
        SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(speeds);

        m_frontLeft.setDesiredState(moduleStates[1]);
        m_frontRight.setDesiredState(moduleStates[0]);
        m_rearLeft.setDesiredState(moduleStates[2]);
        m_rearRight.setDesiredState(moduleStates[3]);
    }

    // For Dylan's testing code. Sets wheels to zero position (within 180 instead of
    // 360 degrees)
    public void zeroOut() {
        m_frontLeft.setDesiredState(new SwerveModuleState());
        m_frontRight.setDesiredState(new SwerveModuleState());
        m_rearLeft.setDesiredState(new SwerveModuleState());
        m_rearRight.setDesiredState(new SwerveModuleState());
    }

    public void zeroDrive() {
        m_frontLeft.zeroDriveMotor();
        m_frontRight.zeroDriveMotor();
        m_rearLeft.zeroDriveMotor();
        m_rearRight.zeroDriveMotor();
    }

    public void updateOdometry() {
        m_odometry.update(
                imu.getRotation2d(),
                getPositions());
    }

    public double getSpeed() {
        return m_frontLeft.getVelocity();
    }

    public void pidTune() {

        // double setpoint = 1.0 / C_WHEELS_CIRCUMFERENCE * 6.75 * 2048 / 10;
        // double driveMotorOutput =
        // m_rearRight.driveMotorPID.calculate(m_rearRight.getVelocity(), setpoint);
        // driveMotorOutput += m_rearRight.m_driveFeedforward.calculate(setpoint);

        // double setpoint = 1;
        // double driveMotorOutput = 0.1;
        //
        // m_rearRight.m_driveMotor.set(ControlMode.PercentOutput,
        // driveMotorOutput/setpoint);
        // m_rearLeft.m_driveMotor.set(ControlMode.PercentOutput,
        // driveMotorOutput/setpoint);
        // m_frontRight.m_driveMotor.set(ControlMode.PercentOutput,
        // driveMotorOutput/setpoint);
        // m_frontLeft.m_driveMotor.set(ControlMode.PercentOutput,
        // driveMotorOutput/setpoint);

        m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
        m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));

        // SmartDashboard.putNumber("rR",
        // m_rearRight.m_driveMotor.getSelectedSensorVelocity() /
        // setpoint);
        // SmartDashboard.putNumber("rL",
        // m_rearLeft.m_driveMotor.getSelectedSensorVelocity() /
        // setpoint);
        // SmartDashboard.putNumber("fR",
        // m_frontRight.m_driveMotor.getSelectedSensorVelocity() /
        // setpoint);
        // SmartDashboard.putNumber("fL",
        // m_frontLeft.m_driveMotor.getSelectedSensorVelocity() /
        // setpoint);

        SmartDashboard.putNumber("rR", m_rearRight.e_Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("rL", m_rearRight.e_Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("fR", m_rearRight.e_Encoder.getAbsolutePosition());
        SmartDashboard.putNumber("fL", m_rearRight.e_Encoder.getAbsolutePosition());
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    public void setSpeedMulti(double num) {
        speedMulti = num;
    }

    public double getSpeedMulti() {
        return speedMulti;
    }

    public void saveEncoderOffsets() {
        m_frontLeft.setEncoderOffset();
        m_frontRight.setEncoderOffset();
        m_rearLeft.setEncoderOffset();
        m_rearRight.setEncoderOffset();
    }

    public void resetAllEncoders() {
        // m_frontLeft.resetEncoderOffset();
        // m_frontRight.resetEncoderOffset();
        // m_rearLeft.resetEncoderOffset();
        // m_rearRight.resetEncoderOffset();
    }
}