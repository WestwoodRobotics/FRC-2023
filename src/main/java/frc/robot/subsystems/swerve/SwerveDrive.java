package frc.robot.subsystems.swerve;

// import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.PortConstants;
import frc.robot.constants.SwerveConstants;


import java.util.Arrays;
// import odometry from wpi


// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
  final Gyro gyro;
  /**
   * Order of swerve modules:
   * <ul>
   *  <li>[0] Front Left</li>
   *  <li>[1] Front Right</li>
   *  <li>[2] Back Left</li>
   *  <li>[3] Back Right</li>
   * </ul>
   */
  private final SwerveModule[] modules = new SwerveModule[4];
  private final Odometry odometry;

  public SwerveDrive() {
    setName("SwerveDrive");
    gyro = new Gyro();

    // Front Left TODO: WHYYYYY ARE THESE SO BAD???
    modules[1] = new SwerveModule(PortConstants.frontLeftDriveMotorPort,
      PortConstants.frontLeftSteerMotorPort, PortConstants.frontLeftEncoderPort, 0);
    // Front Right
    modules[0] = new SwerveModule(PortConstants.frontRightDriveMotorPort,
      PortConstants.frontRightSteerMotorPort, PortConstants.frontRightEncoderPort, 1);
    // Back Left
    modules[3] = new SwerveModule(PortConstants.backLeftDriveMotorPort, PortConstants.backLeftSteerMotorPort, PortConstants.backLeftEncoderPort, 2);
    // Back Right
    modules[2] = new SwerveModule(PortConstants.backRightDriveMotorPort,
      PortConstants.backRightSteerMotorPort, PortConstants.backRightEncoderPort, 3);

    // initialize classes which require Swerve
    odometry = new Odometry(this);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean isFieldRelative) {
    SwerveModuleState[] swerveModuleStates = SwerveConstants.swerveDriveKinematics.toSwerveModuleStates(
      isFieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getYaw())
        : new ChassisSpeeds(xSpeed, ySpeed, rot));

//    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, AutoConstants.kMaxSpeedMetersPerSecond);
    // todo: change based on whether in auton, or teleoperated
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.maxSpeed * 1.2);

    for (int i = 0; i < swerveModuleStates.length; i++) {
      SwerveModuleState swerveModuleState = swerveModuleStates[i];
      SwerveModule module = modules[i];
//      System.out.printf("[%d] %s\n", i, swerveModuleState);
      module.setDesiredState(swerveModuleState);
    }
  }

  public void zeroDrive() {
    for (SwerveModule module : modules) {
      module.zeroDriveMotor();
    }
  }

  public Rotation2d getHeading() {
    return odometry.getHeading();
  }

  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  public SwerveModulePosition[] getPositions() {
    return Arrays.stream(modules).map(SwerveModule::getPosition).toArray(SwerveModulePosition[]::new);
  }

  public void saveEncoderOffsets() {
    for (SwerveModule module : modules) {
      module.setEncoderOffset();
    }
  }

  public void resetAllEncoders() {
    System.out.println("\u001B[32mEncoders reset!\u001B[0m");
    for (SwerveModule module : modules) {
      module.resetEncoders();
    }

    saveEncoderOffsets();
  }

  public void printSteerAngles() {
    System.out.println("\n\u001B[32mFront Left Steer Motor: " + modules[0].getAngleDegrees() + " degrees" + "\n" +
      "Front Right Steer Motor: " + modules[1].getAngleDegrees() + " degrees" + "\n" +
      "Back Left Steer Motor: " + modules[2].getAngleDegrees() + " degrees" + "\n" +
      "Back Right Steer Motor: " + modules[3].getAngleDegrees() + " degrees\u001B[0m");
  }

  public void resetPose(Pose2d pose) {
    odometry.resetOdometry(pose);
  }
  
  public SwerveModule getModule(int id)
  {
    return modules[id];
  }
}
