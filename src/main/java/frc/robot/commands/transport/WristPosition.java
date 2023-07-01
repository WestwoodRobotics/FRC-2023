package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TransportConstants;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;

public class WristPosition extends CommandBase
{
  private Transport m_transport;
  private IntakeModule m_intake;
  private float shoulderPos;
  private double elbowPos;
  private double wristPos;
  private String position;
  //private int mode;
  //boolean validPosition;
  private Timer timer;
  private double startTime;


  public WristPosition(String position, Transport arm, IntakeModule intake) {

    timer = new Timer();
    //mode = intake.getIntakeMode();
    this.position = position;
    m_transport = arm;
    m_intake = intake;
    addRequirements(arm, intake);

    //m_transport.setPosition(newPos);
  }

  @Override
  public void initialize() {

    if(position.equals("UPDATE")) {
      position = m_transport.getPos();
    } else {
      m_transport.setPos(position);
    }

    switch(this.position) {
      case "VERTICAL":
        if (m_intake.getIntakeMode() != 1)
          wristPos = TransportConstants.WRIST_CONE_ROT;
        else
          wristPos = TransportConstants.WRIST_CUBE_ROT;
        break;
      case "HIGH":
        if (m_intake.getIntakeMode() != 1)
          wristPos = TransportConstants.WRIST_START_ROT;
        else
          wristPos = TransportConstants.WRIST_FLIPPED_ROT;
        break;
      case "GROUND":
        if (m_intake.getIntakeMode() == 0) {
          wristPos = TransportConstants.WRIST_CONE_ROT;
        }
        else if (m_intake.getIntakeMode() == 1) {
          wristPos = TransportConstants.WRIST_CUBE_ROT;
        }
        break;
      case "START":
        wristPos = TransportConstants.WRIST_START_ROT;
        break;
      default:
        shoulderPos = 0;
        elbowPos = 0;
        wristPos = 0;
    }

    m_transport.setPos(position);
    SmartDashboard.putString("Transport Position", position);

    timer.reset();
    timer.start();
    startTime = timer.get();

    m_intake.setIntakePower(m_intake.intakeInverted(m_intake.getIntakeMode()) * 0.1);
  }

  @Override
  public void execute() {
    //Wrist
    //Puts percent volts to wrist until it reaches desired ticks
    if (!this.determineWristClose() && (m_transport.getWristMotorPosition() < wristPos))
    {
      m_transport.setWristMotorPower(0.3);
    }
    else if (!this.determineWristClose() && (m_transport.getWristMotorPosition() > wristPos))
    {
      m_transport.setWristMotorPower(-0.3);
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
    m_transport.setShoulderMotorPower(0);
    m_transport.setElbowMotorPower(0);
    m_transport.setWristMotorPower(0);
    m_intake.setIntakePower(0);
  }

  @Override
  public boolean isFinished() {
    return (determineWristFinished()) || ((timer.get() - startTime) > 2);
  }

  //Wrist
  private boolean determineWristFinished() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 0.5);
  }

  private boolean determineWristClose() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 4);
  }
}
