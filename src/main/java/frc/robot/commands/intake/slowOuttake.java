package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.util.Conversions;

public class slowOuttake extends CommandBase {

  IntakeModule intake;
  Timer tim;

  public slowOuttake(IntakeModule i) {
    intake = i;
    addRequirements(i);
    tim = new Timer();
  }

  @Override
  public void initialize() {
    tim.start();
    tim.reset();
    //intake.setIntakePower(-0.75);
    //new WaitCommand(1); //This might lead to problems
    //intake.setIntakePower(0);
  }

  @Override
  public void execute() {
    intake.setIntakePower(-0.75);
  }

  @Override
  public boolean isFinished() {
    return (tim.get() > 0.5);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakePower(0);
  }
  
}
