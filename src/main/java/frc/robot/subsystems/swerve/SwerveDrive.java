package frc.robot.subsystems.swerve;

// import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
  private final SwerveDriveOdometry odometry;
  private final Field2d field;

  public SwerveDrive() {
    setName("SwerveDrive");
    gyro = new Gyro();
    field = new Field2d();

    modules[0] = new SwerveModule(PortConstants.frontLeftDriveMotorPort,
      PortConstants.frontLeftSteerMotorPort, PortConstants.frontLeftEncoderPort, 0);
    // Front Right
    modules[1] = new SwerveModule(PortConstants.frontRightDriveMotorPort,
      PortConstants.frontRightSteerMotorPort, PortConstants.frontRightEncoderPort, 1);
    // Back Left
    modules[2] = new SwerveModule(PortConstants.backLeftDriveMotorPort, PortConstants.backLeftSteerMotorPort, PortConstants.backLeftEncoderPort, 2);
    // Back Right
    modules[3] = new SwerveModule(PortConstants.backRightDriveMotorPort,
      PortConstants.backRightSteerMotorPort, PortConstants.backRightEncoderPort, 3);

    // initialize classes which require Swerve
    odometry = new SwerveDriveOdometry(
      SwerveConstants.swerveDriveKinematics,
      getHeading(),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      },
      new Pose2d(5, 5, new Rotation2d())
    );
    SmartDashboard.putData("Field", field);
    
  }

  public void periodic() {
    odometry.update(
      getHeading(),
      new SwerveModulePosition[] {
        modules[0].getPosition(),
        modules[1].getPosition(),
        modules[2].getPosition(),
        modules[3].getPosition()
      }
    );
    field.setRobotPose(this.getPoseMeters());
    // SmartDashboard.putNumber("Front left pos", modules[0].getPosition().distanceMeters);
    // SmartDashboard.putNumber("Front right pos", modules[1].getPosition().distanceMeters);
    // SmartDashboard.putNumber("Back left pos", modules[2].getPosition().distanceMeters);
    // SmartDashboard.putNumber("Back right pos", modules[3].getPosition().distanceMeters);
    // SmartDashboard.putNumber("Front left ang", modules[0].getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Front right ang", modules[1].getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Back left ang", modules[2].getPosition().angle.getDegrees());
    // SmartDashboard.putNumber("Back right ang", modules[3].getPosition().angle.getDegrees());
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
      //SmartDashboard.putNumber("mod thinky", modules[3].getVelocity(0));
      SmartDashboard.putNumber("other thinky", modules[0].getVelocity(0));
    }
  }

  public void zeroDrive() {
    for (SwerveModule module : modules) {
      module.zeroDriveMotor();
    }
  }

  public void zeroTurn()
  {
    for (SwerveModule module : modules)
    {
      module.zeroTurnMotor();
    }
  }

  public boolean resetGyro(){
    gyro.zeroGyro();
    return true;
  }

  public Rotation2d getHeading() {
    return gyro.getYaw().unaryMinus();
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
    // odometry.resetPosition(pose);
  }
  
  public SwerveModule getModule(int id)
  {
    return modules[id];
  }

  public void setModuleStatesDirectly(SwerveModuleState[] desiredStates)
  {
    for (int i = 0; i < desiredStates.length; i++) {
      SwerveModuleState desiredState = desiredStates[i];
      SwerveModule module = modules[i];
      module.setDesiredState(desiredState);
      System.out.printf("[%d] %s\n", i, desiredState);
      SmartDashboard.putString("Swerve state", desiredState.toString());
    }
  }

  public void setForwardTurn()
  {
    SwerveModuleState [] list = new SwerveModuleState [4];
    for (int i = 0; i < list.length; i++)
    {
      list[i] = new SwerveModuleState(0, Rotation2d.fromDegrees(90));
    }
    this.setModuleStatesDirectly(list);
  }

  public double getAverageDriveEncoderPositions(){
    double sum = 0;
    for (SwerveModule module : modules) {
      sum += module.getDriveMotorEncoderValue();
    }
    return sum / modules.length;
  }
}
