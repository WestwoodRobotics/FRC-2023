package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;

public class ArmPositions extends CommandBase
{
  private Transport m_transport;
  private IntakeModule m_intake;
  private float shoulderPos;
  private double elbowPos;
  private double wristPos;
  private String currentPos;
  //boolean validPosition;
  private Timer timer;
  private double startTime;


  public ArmPositions(double shoulderPosition, double elbowPosition, double wristPosition, Transport arm, IntakeModule intake) {

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

    this.shoulderPos = (float) shoulderPosition;
    this.elbowPos = elbowPosition;
    this.wristPos = wristPosition;

    m_transport = arm;
    m_intake = intake;
    addRequirements(arm, intake);

    //m_transport.setPosition(newPos);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    startTime = timer.get();

    m_intake.setIntakePower(0.1);
  }

  @Override
  public void execute() {

    //SmartDashboard.putString("Current Position", currentPos);
    m_transport.setShoulderMotorPosition(shoulderPos);
    //ELbow
    //Puts percent volts to elbow until it reaches desired ticks
    if (!this.determineElbowClose() && (m_transport.getElbowMotorPosition() < elbowPos))
    {
      m_transport.setElbowMotorPower(0.8);
    }
    else if (!this.determineElbowClose() && (m_transport.getElbowMotorPosition() > elbowPos))
    {
      m_transport.setElbowMotorPower(-0.8);
    }
    //decreases power when it is close to desired ticks to prevent rapidly going to 0 volts
    else if (!this.determineElbowFinished() && (m_transport.getElbowMotorPosition() < elbowPos))
    {
      m_transport.setElbowMotorPower(0.4);
    }
    else if (!this.determineElbowFinished() && (m_transport.getElbowMotorPosition() > elbowPos))
    {
      m_transport.setElbowMotorPower(-0.4);
    }
    else if (this.determineElbowFinished())
    {
      m_transport.setElbowMotorPower(0);
    }

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
    m_transport.setShoulderMotorPower(0);
    m_transport.setElbowMotorPower(0);
    m_transport.setWristMotorPower(0);
    m_intake.setIntakePower(0);
  }

  @Override
  public boolean isFinished() {
    return (determineShoulderFinished() && determineElbowFinished() && determineWristFinished()) || ((timer.get() - startTime) > 4);
  }

  //Shoulder
  private boolean determineShoulderFinished() { //Should these all be .75 rotations? Different Gear Ratios?
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 0.5);
  }

  private boolean determineShoulderClose() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 4);
  }

  //Elbow
  private boolean determineElbowFinished() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 0.5);
  }

  private boolean determineElbowClose() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 4);
  }

  //Wrist
  private boolean determineWristFinished() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 0.5);
  }

  private boolean determineWristClose() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 4);
  }
}
