package frc.robot.util;

public class Conversions {

  /**
   * @param counts    Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Degrees of Rotation of Mechanism
   */
  public static double falconToDegrees(double counts, double gearRatio) {
    return counts * (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param degrees   Degrees of rotation of Mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Falcon Counts
   */
  public static double degreesToFalcon(double degrees, double gearRatio) {
    return degrees / (360.0 / (gearRatio * 2048.0));
  }

  /**
   * @param counts    Falcon Counts
   * @param gearRatio Gear Ratio between Falcon and Mechanism
   * @return Radians of Rotation of Mechanism
   */
  public static double FalconToRadians(double counts, double gearRatio) {
    return counts * (2 * Math.PI / (gearRatio * 2048.0));
  }

  /**
   * @param velocityCounts Falcon Velocity Counts
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return RPM of Mechanism
   */
  public static double falconToRPM(double velocityCounts, double gearRatio) {
    double motorRPM = velocityCounts * (600.0 / 2048.0);
    return motorRPM / gearRatio;
  }

  /**
   * @param RPM       RPM of mechanism
   * @param gearRatio Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon sensor ticks
   */
  public static double RPMToFalcon(double RPM, double gearRatio) {
    double motorRPM = RPM * gearRatio;
    return motorRPM * (2048.0 / 600.0);
  }

  /**
   * @param velocitycounts Falcon Velocity Counts
   * @param circumference  Circumference of Wheel
   * @param gearRatio      Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Wheel Meters Per Second
   */
  public static double falconToMPS(double velocitycounts, double circumference, double gearRatio) {
    double wheelRPM = falconToRPM(velocitycounts, gearRatio);
    double wheelMPS = (wheelRPM * circumference) / 60;
    return wheelMPS;
  }

  /**
   * @param velocity      Velocity MPS
   * @param circumference Circumference of Wheel
   * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Falcon Velocity Counts
   */
  public static double MPSToFalcon(double velocity, double circumference, double gearRatio) {
    double wheelRPM = ((velocity * 60) / circumference);
    double wheelVelocity = RPMToFalcon(wheelRPM, gearRatio);
    return wheelVelocity;
  }

  /**
   * @param falconTicks   Falcon Encoder Position
   * @param circumference Circumference of Wheel
   * @param gearRatio     Gear Ratio between Falcon and Mechanism (set to 1 for Falcon RPM)
   * @return Meters traveled
   */
  public static double FalconToMeters(double falconTicks, double circumference, double gearRatio) {
    double wheelRevs = (falconTicks / 2048.0) / gearRatio;
    double meters = wheelRevs * circumference;
    return meters;
  }

  public static double deadZoneSquare(double input, double deadband) {
    if (Math.abs(input) < deadband) {
      return 0;
    }
    else {
      return Math.copySign(Math.pow(input, 2), input);
    }
  }
}
