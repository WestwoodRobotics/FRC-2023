package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;

public class IntakeModes extends CommandBase
{
  private Transport m_transport;
  private IntakeModule m_intake;
  private String currentPos;
  //boolean validPosition;
  private Timer timer;
  private double startTime;
  private static int intakeIncr;
  private boolean done = false;


  public IntakeModes(Transport transport, IntakeModule intake) {

    //checks that the positions transitions do no result in going over height limit
    /*
    currentPos = arm.getPosition();
    if ((currentPos.equals("HIGH") || currentPos.equals("MID")) && (!newPos.equals("START"))) {
      validPosition = false;
    } else {
      validPosition = true;
    }
    */

    timer = new Timer();

    m_transport = transport;
    m_intake = intake;
    addRequirements(transport, intake);

    //m_transport.setPosition(newPos);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startTime = timer.get();


  }

  @Override
  public void execute() {
    intakeIncr++;
    done = true;

  }

  public int getIntakeIncrement(){
    return intakeIncr;
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return (done || timer.get() - startTime > 2);
  }
}
