package frc.robot.subsystems.swerve;

//import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
import frc.robot.Constants.*;

import java.io.*;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Paths;
import java.util.List;


// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
  private final SwerveModule frontLeftModule;
  private final SwerveModule frontRightModule;
  private final SwerveModule backLeftModule;
  private final SwerveModule backRightModule;
  private double turnEncoderOffsets[];

  // private final SwerveDriveKinematics m_kinematics = new
  // SwerveDriveKinematics(DriveConstants.frontRight, DriveConstants.frontLeft,
  // DriveConstants.backRight, DriveConstants.backLeft);

  protected Gyro gyro;
  public Odometry odometry;

  public SwerveDrive() {
    setName("SwerveDrive");
    gyro = new Gyro();
    turnEncoderOffsets = new double[4];
    // initialize swerve mods (possibly move into a list for conciseness eventually)
    frontLeftModule = new SwerveModule(PortConstants.kFrontLeftDriveMotorPort, PortConstants.kFrontLeftSteerMotorPort,
        false, false, PortConstants.kFrontLeftCANCoderPort, 0, 0);
    frontRightModule = new SwerveModule(PortConstants.kFrontRightDriveMotorPort,
        PortConstants.kFrontRightSteerMotorPort, false, false, PortConstants.kFrontRightCANCoderPort, 0, 1);
    backLeftModule = new SwerveModule(PortConstants.kBackLeftDriveMotorPort, PortConstants.kBackLeftSteerMotorPort,
        false, false, PortConstants.kBackLeftCANCoderPort, 0, 2);
    backRightModule = new SwerveModule(PortConstants.kBackRightDriveMotorPort, PortConstants.kBackRightSteerMotorPort,
        false, false, PortConstants.kBackRightCANCoderPort, 0, 3);

    // initialize classes which require Swerve
    odometry = new Odometry(this);
    loadEncoderOffsets();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getYaw())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));

    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, AutoConstants.kMaxSpeedMetersPerSecond);
    frontRightModule.setDesiredState(swerveModuleStates[0]);
    frontLeftModule.setDesiredState(swerveModuleStates[1]);
    backLeftModule.setDesiredState(swerveModuleStates[2]);
    backRightModule.setDesiredState(swerveModuleStates[3]);
  }

  public void zeroDrive() {
    frontLeftModule.zeroDriveMotor();
    frontRightModule.zeroDriveMotor();
    backLeftModule.zeroDriveMotor();
    backRightModule.zeroDriveMotor();
  }

  public Rotation2d getHeading() {
    return odometry.getHeading();
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = frontLeftModule.getPosition();
    positions[1] = frontRightModule.getPosition();
    positions[2] = backLeftModule.getPosition();
    positions[3] = backRightModule.getPosition();
    return positions;
  }

  public void setStates(SwerveModuleState[] states) {
    frontLeftModule.setDesiredState(states[0]);
    frontRightModule.setDesiredState(states[1]);
    backLeftModule.setDesiredState(states[2]);
    backRightModule.setDesiredState(states[3]);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    states[0] = frontLeftModule.getState();
    states[1] = frontRightModule.getState();
    states[2] = backLeftModule.getState();
    states[3] = backRightModule.getState();
    return states;
  }

  public void loadEncoderOffsets() {
    try {
        //Reads all the lines of the file, and ignores any data beyond the Standard Character Set
        List<String> lines = Files.readAllLines(Paths.get(FilePathConstants.steerEncoderOffsetSavesPath), StandardCharsets.UTF_8);
        turnEncoderOffsets = lines.stream().limit(4).mapToDouble(Double::parseDouble).toArray();
    } catch (IOException | NumberFormatException e) {
        System.out.println(
            "\u001b[31;1mFailed to read turn encoder offsets from file, please align wheels manually, then reset encoders.\u001b[0m");
    }
    frontLeftModule.setEncoderOffset(turnEncoderOffsets[0]);
    frontRightModule.setEncoderOffset(turnEncoderOffsets[1]);
    backLeftModule.setEncoderOffset(turnEncoderOffsets[2]);
    backRightModule.setEncoderOffset(turnEncoderOffsets[3]);
  }

  public void calculateEncoderOffsets() {
    turnEncoderOffsets[0] = frontLeftModule.calculateEncoderOffset();
    turnEncoderOffsets[1] = frontRightModule.calculateEncoderOffset();
    turnEncoderOffsets[2] = backLeftModule.calculateEncoderOffset();
    turnEncoderOffsets[3] = backRightModule.calculateEncoderOffset();
  }

  public void setDisabled(boolean disabled) {
    frontLeftModule.setDisabled(disabled);
    frontRightModule.setDisabled(disabled);
    backLeftModule.setDisabled(disabled);
    backRightModule.setDisabled(disabled);
  }


  public void saveEncoderOffsets() {
    try (BufferedWriter writer = new BufferedWriter(new FileWriter(FilePathConstants.steerEncoderOffsetSavesPath))) {
      for (double encoderOffset : turnEncoderOffsets) {
          writer.write(Double.toString(encoderOffset));
          writer.newLine();
      }
    } catch (IOException e) {
        System.out.println("\u001b[31;1mFailed to write turn encoder offsets to file.\u001b[0m");
    }
  }

  public void resetAllEncoders() {
    frontLeftModule.resetEncoders();
    frontRightModule.resetEncoders();
    backLeftModule.resetEncoders();
    backRightModule.resetEncoders();

  }

  public void resetPose(Pose2d pose) {
    odometry.resetOdometry(pose);
  }
}
