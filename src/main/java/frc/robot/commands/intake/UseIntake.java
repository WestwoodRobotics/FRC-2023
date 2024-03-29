package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TransportConstants;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

public class UseIntake extends CommandBase {

  IntakeModule intake;
  XboxController primaryController, secondaryController;

  public UseIntake(XboxController primaryController, XboxController secondaryController, IntakeModule i) {
    this.primaryController = primaryController;
    this.secondaryController = secondaryController;
    intake = i;
    addRequirements(i);
  }

  @Override
  public void execute() {
    double leftTriggerMagnitude = Math.max(primaryController.getLeftTriggerAxis(), secondaryController.getLeftTriggerAxis());
    double rightTriggerMagnitude = Math.max(primaryController.getRightTriggerAxis(), secondaryController.getRightTriggerAxis());
    leftTriggerMagnitude = Conversions.deadZoneSquare(leftTriggerMagnitude, 0.1);
    rightTriggerMagnitude = Conversions.deadZoneSquare(rightTriggerMagnitude, 0.1);

    if(secondaryController.getLeftBumper()){
      intake.setIntakePower(-intake.intakeInverted(intake.getIntakeMode()) * 0.25);
    } else {
      intake.setIntakePower(0.75 * intake.intakeInverted(intake.getIntakeMode()) * (rightTriggerMagnitude - leftTriggerMagnitude));
    }
  }
}
