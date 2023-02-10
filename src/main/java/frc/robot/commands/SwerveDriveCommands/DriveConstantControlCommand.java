package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.DetermineDriveSpeed;
import frc.robot.subsystems.swerve.SwerveDrive;

import static frc.robot.Constants.AutoConstants;
import static frc.robot.Constants.OIConstants;

public class DriveConstantControlCommand extends CommandBase {

  private final SwerveDrive m_swerveDrive;
  private final XboxController controller;
  private final DetermineDriveSpeed limJoystickLeft = new DetermineDriveSpeed(0.05);
  private final DetermineDriveSpeed limJoystickRight = new DetermineDriveSpeed(0.05);

  public DriveConstantControlCommand(SwerveDrive swerveDrive, XboxController controller) {
    m_swerveDrive = swerveDrive;
    this.controller = controller;

    addRequirements(swerveDrive);
  }

  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftX, leftY, rightX;

    leftX = controller.getLeftX();
    leftY = -controller.getLeftY();
    rightX = controller.getRightX();

    // Find radii for controller dead-zones (circular)
    double leftRadius = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
    double rightRadius = Math.abs(rightX);

    // add input curves
    leftX = Math.pow(leftX, 3);
    leftY = Math.pow(leftY, 3);
    rightX = Math.pow(rightX, 3);

    // apply deadzones
    if (leftRadius < OIConstants.kDeadzoneCircle) {
      leftX = 0;
      leftY = 0;
    }

    if (rightRadius < OIConstants.kDeadzoneCircle) {
      rightX = 0;
    }

    limJoystickLeft.compute(leftX, leftY);
    limJoystickRight.compute(rightX, 0);

    leftX = limJoystickLeft.xSpeed;
    leftY = limJoystickLeft.ySpeed;
    rightX = limJoystickRight.xSpeed;

    // apply max speeds
    leftX *= AutoConstants.kMaxSpeedMetersPerSecond;
    leftY *= AutoConstants.kMaxSpeedMetersPerSecond;
    rightX *= AutoConstants.kMaxAngularSpeedRadiansPerSecond;

    // if left stick is active, drive in that direction
    if (leftRadius >= OIConstants.kDeadzoneRectangle) {
      m_swerveDrive.drive(leftX, leftY, rightX, false);
    } else if (rightRadius >= OIConstants.kDeadzoneRectangle) {
      // otherwise, if right stick is active, turn in that direction
      m_swerveDrive.drive(0, 0, rightX, false);
    } else {
      // otherwise, stop drive motors
      m_swerveDrive.zeroDrive();
    }
  }

  private double checkDeadzone(double val) {
    // zeros if within deadzone rectangle
    if (Math.abs(val) < OIConstants.kDeadzoneRectangle) return 0;
    // squares the value to decrease sensitivity
    // else if (val < 0) return -Math.pow(val, 3);
    return Math.pow(val, 3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should never end in teleop
    return false;
  }
}
