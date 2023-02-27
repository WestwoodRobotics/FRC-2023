package frc.robot.commands.SwerveDriveCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveSpeed;
import frc.robot.subsystems.swerve.SwerveDrive;

import static frc.robot.Constants.DriveConstants.maxAngularSpeed;
import static frc.robot.Constants.DriveConstants.maxSpeed;

public class DriveConstantControlCommand extends CommandBase {

  private final SwerveDrive m_swerveDrive;
  private final DriveSpeed limJoystickLeft = new DriveSpeed(0.05);
  private final DriveSpeed limJoystickRight = new DriveSpeed(0.05);
  private XboxController controller;

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

    leftX = -controller.getLeftX();
    leftY = -controller.getLeftY();
    rightX = controller.getRightX();


    // Find radii for controller dead-zones (circular)
    double leftRadius = Math.sqrt(Math.pow(leftX, 2) + Math.pow(leftY, 2));
    double rightRadius = Math.abs(rightX);

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
    leftX *= maxSpeed;
    leftY *= maxSpeed;
    rightX *= maxAngularSpeed;

    // if left stick is active, drive in that direction
    if (leftRadius >= OIConstants.kDeadzoneRectangle) {
      m_swerveDrive.drive(leftX, leftY, rightX, false);
    } else if (rightRadius >= OIConstants.kDeadzoneRectangle) {
      // otherwise, if right stick is active, turn in that direction
      m_swerveDrive.drive(0, 0, rightX, false);
    } else if (controller.getAButton()) {
      // otherwise, if A is pressed, turn the wheels right slowly
      m_swerveDrive.drive(0, 0, 0.1, false);
    } else {
      // otherwise, stop drive motors
      m_swerveDrive.zeroDrive();
    }
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
