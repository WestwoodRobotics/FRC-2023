package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AutoConstants {

  public static final double maxVelocity = Units.inchesToMeters(20);
  public static final double maxAcceleration = Units.inchesToMeters(20);
  public static final double maxAngularVelocity = maxAcceleration / 10;
  public static final double maxAngularAcceleration = Math.PI / 4;

  public static final class PID {
    // Proportional gain for the X controller
    public static final double controllerX = 1.5;
    // Proportional gain for the Y controller
    public static final double controllerY = 1.5;
    // Proportional gain for the Theta controller
    public static final double controllerTheta = 3;
  }

  // Constraints for the Theta controller
  public static final TrapezoidProfile.Constraints thetaControllerConstraints = //
    new TrapezoidProfile.Constraints(maxAngularVelocity,
      maxAngularAcceleration);
}
