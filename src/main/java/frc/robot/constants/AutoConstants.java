package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class AutoConstants {

  public static final double maxVelocity = 2; // meters per second
  public static final double maxAcceleration = 2; // meters per second squared
  public static final double maxAngularVelocity = Math.PI / 2; // radians per second
  public static final double maxAngularAcceleration = Math.PI / 2; // radians per second squared

  public static final class PID {
    // Proportional gain for the X controller
    public static final double kPControllerX = 2,
                               kDControllerX = .2;
    // Proportional gain for the Y controller
    public static final double kPControllerY = 2,
                               kDControllerY = .20;                             ;
    // Proportional gain for the Theta controller
    public static final double kPControllerTheta = 3,
                               kDControllerTheta = .01;
  }

  // Constraints for the Theta controller
  public static final TrapezoidProfile.Constraints thetaControllerConstraints = //
    new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration);
} 
