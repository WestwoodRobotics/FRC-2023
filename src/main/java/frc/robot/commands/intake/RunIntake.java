package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.util.Conversions;

public class RunIntake extends CommandBase {

  IntakeModule intake;
  Timer tim;
  double power;
  double modifier;

  public RunIntake(IntakeModule i, double modifier) {
    
    this.modifier = modifier;
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
    intake.setIntakePower(-1 * modifier * intake.intakeInverted(intake.getIntakeMode()) * 0.25);
  }

  @Override
  public boolean isFinished() {
    return (tim.get() > 1);
  }

  @Override
  public void end(boolean interrupted) {
    intake.setIntakePower(0);
  }
  
}
