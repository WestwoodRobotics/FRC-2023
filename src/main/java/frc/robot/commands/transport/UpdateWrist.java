package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TransportConstants;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;

public class UpdateWrist extends CommandBase
{
  private Transport m_transport;
  private IntakeModule m_intake;
  private double wristPos;
  private String position;
  //private int mode;
  //boolean validPosition;
  private Timer timer;
  private double startTime;


  public UpdateWrist(Transport arm, IntakeModule intake) {

    timer = new Timer();
    //mode = intake.getIntakeMode();

    m_transport = arm;
    m_intake = intake;
    addRequirements(arm, intake);

    //m_transport.setPosition(newPos);
  }

  @Override
  public void initialize() {

    switch(m_transport.getPos()) {
      case "HIGH":
        if (m_intake.getIntakeMode() != 2)
          wristPos = TransportConstants.WRIST_START_ROT;
        else
          wristPos = TransportConstants.WRIST_FLIPPED_ROT;
        break;
      case "GROUND":
        if (m_intake.getIntakeMode() == 0)
          wristPos = TransportConstants.WRIST_FLIPPED_ROT;
        else if (m_intake.getIntakeMode() == 1)
          wristPos = TransportConstants.WRIST_HALF_ROT;
        else if (m_intake.getIntakeMode() == 2)
          wristPos = TransportConstants.WRIST_START_ROT;
        break;
      default:
    }

    //m_transport.setPos(position);
    //SmartDashboard.putString("Transport Position", position);

    timer.reset();
    timer.start();
    startTime = timer.get();

    m_intake.setIntakePower(0.1);
  }

  @Override
  public void execute() {
    //Wrist
    //Puts percent volts to wrist until it reaches desired ticks
    if (!this.determineWristClose() && (m_transport.getWristMotorPosition() < wristPos))
    {
      m_transport.setWristMotorPower(0.4);
    }
    else if (!this.determineWristClose() && (m_transport.getWristMotorPosition() > wristPos))
    {
      m_transport.setWristMotorPower(-0.4);
    }
    //decreases power when it is close to desired ticks to prevent rapidly going to 0 volts
    else if (!this.determineWristFinished() && (m_transport.getWristMotorPosition() < wristPos))
    {
      m_transport.setWristMotorPower(0.2);
    }
    else if (!this.determineWristFinished() && (m_transport.getWristMotorPosition() > wristPos))
    {
      m_transport.setWristMotorPower(-0.2);
    }
    else if (this.determineWristFinished())
    {
      m_transport.setWristMotorPower(0);
    }
    //m_transport.setShoulderMotorPosition(shoulderPos);
    //m_transport.setElbowMotorPosition(elbowPos);
  }

  @Override
  public void end(boolean interrupted) {
    m_transport.setWristMotorPower(0);
    m_intake.setIntakePower(0);
  }

  @Override
  public boolean isFinished() {
    return (determineWristFinished()) || ((timer.get() - startTime) > 4);
  }
  //Wrist
  private boolean determineWristFinished() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 0.5);
  }

  private boolean determineWristClose() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 4);
  }
}
