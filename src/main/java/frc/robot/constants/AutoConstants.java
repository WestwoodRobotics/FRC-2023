package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AutoConstants {

  public static final double maxVelocity = 1; // meters per second
  public static final double maxAcceleration = 2; // meters per second squared
  public static final double maxAngularVelocity = Math.PI / 2;
  public static final double maxAngularAcceleration = Math.PI / 2;

  public static final class PID {
    // Proportional gain for the X controller
    public static final double kPControllerX = 1.5,
                               kDControllerX = 0;
    // Proportional gain for the Y controller
    public static final double kPControllerY = 1.5,
                               kDControllerY = 0;
    // Proportional gain for the Theta controller
    public static final double kPControllerTheta = 3,
                               kDControllerTheta = 0;
  }

  // Constraints for the Theta controller
  public static final TrapezoidProfile.Constraints thetaControllerConstraints = //
    new TrapezoidProfile.Constraints(maxAngularVelocity,
      maxAngularAcceleration);
}
