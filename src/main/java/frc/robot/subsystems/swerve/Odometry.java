// Adapted from Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Odometry.java

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriveConstants;

public class Odometry extends SubsystemBase {

  public SwerveDrivePoseEstimator swerveOdometry;
  private SwerveDrive swerve;

  public Odometry(SwerveDrive s) {
    swerve = s;
    swerveOdometry = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics, swerve.gyro.getYaw(), swerve.getPositions(), new Pose2d());
  }

  public void update() {
    swerveOdometry.update(swerve.gyro.getYaw(), swerve.getPositions());
  }

  public Pose2d getPoseMeters() {
    return swerveOdometry.getEstimatedPosition();
  }


  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(swerve.gyro.getYaw(), swerve.getPositions(), pose);
  }


  public Translation2d getTranslationMeters() {
    return getPoseMeters().getTranslation();
  }

  public Rotation2d getHeading() {
    return getPoseMeters().getRotation();
  }

  public void resetHeading(Rotation2d newHeading) {
    resetOdometry(new Pose2d(getTranslationMeters(), newHeading));
  }

  public void addVisionMeasurement(Pose2d visionPose, double visionTimestamp) {
    swerveOdometry.addVisionMeasurement(visionPose, visionTimestamp);
  }

  @Override
  public void periodic() {
    this.update();
  }
}
