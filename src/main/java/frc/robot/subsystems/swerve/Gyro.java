// Adapted from team Spectrum 3847
// https://github.com/Spectrum3847/2023-X-Ray/blob/main/src/main/java/frc/robot/swerve/Gyro.java

package frc.robot.subsystems.swerve;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class Gyro extends SubsystemBase {
  public WPI_Pigeon2 pigeon;
  public Rotation2d yawOffset = new Rotation2d(0);

  /**
   * Creates a new Gyro, which is a wrapper for the Pigeon IMU and stores an offset so we don't
   * have to directly zero the gyro
   */
  public Gyro() {
    pigeon = new WPI_Pigeon2(PortConstants.pigeonPort);
    pigeon.configFactoryDefault();
    zeroGyro();
  }

  /**
   * Zero the gyro
   */
  public void zeroGyro() {
    setGyroDegrees(0);
  }

  /**
   * Set the gyro yawOffset to a specific angle
   *
   * @param degrees new angle in degrees
   */
  public void setGyroDegrees(double degrees) {
    yawOffset = getRawYaw().minus(Rotation2d.fromDegrees(degrees));
  }

  /**
   * Get the yaw of the robot in Rotation2d
   *
   * @return the yaw of the robot in Rotation2d
   */
  public Rotation2d getYaw() {
    return getRawYaw().minus(yawOffset).times(-1);
  }

  /**
   * Get the raw yaw of the robot in Rotation2d without using the yawOffset
   *
   * @return the raw yaw of the robot in Rotation2d
   */
  public Rotation2d getRawYaw() {
    return Rotation2d.fromDegrees(pigeon.getYaw());
  }

  public double getPitch(){
    return pigeon.getPitch();
  }

  public void resetRoll(){
    pigeon.reset();
  }

  public double getRoll(){
    return pigeon.getRoll();
  }
}
