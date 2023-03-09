package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;

public class ArmPositionsV2 extends CommandBase {

  Transport m_transport;
  double shoulderPos;
  double elbowPos;


  public ArmPositionsV2(double shoulderPosition, double elbowPosition, Transport arm) {
    this.shoulderPos = shoulderPosition;
    this.elbowPos = elbowPosition;

    m_transport = arm;
    addRequirements(arm);
  }
  
  @Override
  public void execute() {
    System.out.println(m_transport.getShoulderMotorPosition());
    if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(0.4);
    } 
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-0.4);
    } 
    else if (this.determineShoulderFinished()) 
    {
      m_transport.setShoulderMotorPower(0);
    }


    if (!this.determineElbowFinished() && (m_transport.getElbowMotorPosition() < elbowPos)) 
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
    //m_transport.setShoulderMotorPosition(shoulderPos);
    //m_transport.setElbowMotorPosition(elbowPos);
  }

  @Override
  public void end(boolean interrupted) {
    m_transport.setShoulderMotorPower(0);
    m_transport.setElbowMotorPower(0);
  }

  @Override
  public boolean isFinished() {
    return determineShoulderFinished() && determineElbowFinished();
  }

  private boolean determineShoulderFinished() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 1750) ;
  }

  private boolean determineElbowFinished() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 1750) ;
  }
}

