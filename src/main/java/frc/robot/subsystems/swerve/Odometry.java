// Adapted from Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Odometry.java

package frc.robot.subsystems.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;

import static frc.robot.Constants.DriveConstants;

public class Odometry {

    public SwerveDriveOdometry swerveOdometry;
    private SwerveDrive swerve;

    public Odometry(SwerveDrive s) {
        swerve = s;
        swerveOdometry = new SwerveDriveOdometry(
                DriveConstants.kDriveKinematics, swerve.gyro.getYaw(), swerve.getPositions());
    }

    public SwerveDriveOdometry getSwerveDriveOdometry() {
        return swerveOdometry;
    }

    public void update() {
        swerveOdometry.update(swerve.gyro.getYaw(), swerve.getPositions());
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(swerve.gyro.getYaw(), swerve.getPositions(), pose);
    }

    public Pose2d getPoseMeters() {
        return swerveOdometry.getPoseMeters();
    }

    public Translation2d getTranslationMeters() {
        return swerveOdometry.getPoseMeters().getTranslation();
    }

    public Rotation2d getHeading() {
        return getPoseMeters().getRotation();
    }

    public void resetHeading(Rotation2d newHeading) {
        resetOdometry(new Pose2d(getTranslationMeters(), newHeading));
    }
}
