package frc.robot.subsystems.Chassis;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;


// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
  // Initialize the 4 swerve modules with the appropriate CAN IDs, inversion states, and encoder offsets
  SwerveModule frontLeftModule = new SwerveModule(PortConstants.kFrontLeftDriveMotorPort, PortConstants.kFrontLeftSteerMotorPort, false, false, PortConstants.kFrontLeftCANCoderPort, 0, false);
  SwerveModule frontRightModule = new SwerveModule(PortConstants.kFrontRightDriveMotorPort, PortConstants.kFrontRightSteerMotorPort, false, false, PortConstants.kFrontRightCANCoderPort, 0, false);
  SwerveModule backLeftModule = new SwerveModule(PortConstants.kBackLeftDriveMotorPort, PortConstants.kBackLeftSteerMotorPort, false, false, PortConstants.kBackLeftCANCoderPort, 0, false);
  SwerveModule backRightModule = new SwerveModule(PortConstants.kBackRightDriveMotorPort, PortConstants.kBackRightSteerMotorPort, false, false, PortConstants.kBackRightCANCoderPort, 0, false);


  private final WPI_Pigeon2 imu = new WPI_Pigeon2(PortConstants.kPigeonPort);

  private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(DriveConstants.frontRight, DriveConstants.frontLeft, DriveConstants.backRight, DriveConstants.backLeft);


  private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, imu.getRotation2d(), null); //TODO: Find out what the null should be

  private final double speedMulti = 1;

  public SwerveDrive() {
    imu.reset();
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

    SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
      fieldRelative
        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, imu.getRotation2d())
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
}
