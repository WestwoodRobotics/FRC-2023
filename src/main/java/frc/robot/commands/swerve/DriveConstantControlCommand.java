package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ControllerConstants;
import frc.robot.subsystems.swerve.DriveSpeed;
import frc.robot.subsystems.swerve.SwerveDrive;

import static frc.robot.constants.DriveConstants.maxAngularSpeed;
import static frc.robot.constants.DriveConstants.maxSpeed;

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
    if (leftRadius < ControllerConstants.deadzoneCircle) {
      leftX = 0;
      leftY = 0;
    }

    if (rightRadius < ControllerConstants.deadzoneCircle) {
      rightX = 0;
    }

    limJoystickLeft.compute(leftX, leftY);
    limJoystickRight.compute(rightX, 0);

//    System.out.printf("X: %f - Y: %f  -- Limited X: %f - Limited Y: %f%n", leftX, leftY, limJoystickLeft.xSpeed, limJoystickLeft.ySpeed);

    leftX = limJoystickLeft.xSpeed;
    leftY = limJoystickLeft.ySpeed;
    rightX = limJoystickRight.xSpeed;

    // apply max speeds
    leftX *= maxSpeed;
    leftY *= maxSpeed;
    rightX *= maxAngularSpeed;

    m_swerveDrive.drive((leftRadius >= ControllerConstants.deadzoneRectangle ? leftX : 0),
                        (leftRadius >= ControllerConstants.deadzoneRectangle ? leftY : 0),
                        (rightRadius >= ControllerConstants.deadzoneRectangle ? rightX : 0),
                        true);
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
