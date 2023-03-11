package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.util.Conversions;

public class slowOuttake extends CommandBase {

  IntakeModule intake;

  public slowOuttake(IntakeModule i) {
    intake = i;
    addRequirements(i);
  }

  @Override
  public void initialize() {
    intake.setIntakePower(-0.25);
    new WaitCommand(1); //This might lead to problems
    intake.setIntakePower(0);
  }

  @Override
  public boolean isFinished() {
    return true;
  }
  
}
