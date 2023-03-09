package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;

public class ArmPositions extends CommandBase {

  Transport m_transport;
  double shoulderPos;
  double elbowPos;


  public ArmPositions(double shoulderPosition, double elbowPosition, Transport arm) {
    this.shoulderPos = shoulderPosition;
    this.elbowPos = elbowPosition;

    m_transport = arm;
    addRequirements(arm);
  }
  
  @Override
  public void execute() {
    System.out.println(m_transport.getShoulderMotorPosition());
    if (!this.determineShoulderClose() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(0.5);
    } 
    else if (!this.determineShoulderClose() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-0.5);
    } 
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(0.2);
    }
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-0.2);
    } 
    else if (this.determineShoulderFinished()) 
    {
      m_transport.setShoulderMotorPower(0);
    }


    if (!this.determineElbowClose() && (m_transport.getElbowMotorPosition() < elbowPos)) 
    {
      m_transport.setElbowMotorPower(0.8);
    } 
    else if (!this.determineElbowClose() && (m_transport.getElbowMotorPosition() > elbowPos)) 
    {
      m_transport.setElbowMotorPower(-0.8);
    } 
    else if (!this.determineElbowFinished() && (m_transport.getElbowMotorPosition() < elbowPos)) 
    {
      m_transport.setElbowMotorPower(0.3);
    } 
    else if (!this.determineElbowFinished() && (m_transport.getElbowMotorPosition() > elbowPos)) 
    {
      m_transport.setElbowMotorPower(-0.3);
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
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 1250) ;
  }
  
  private boolean determineShoulderClose() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 10000) ;
  }

  private boolean determineElbowFinished() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 1250) ;
  }

  private boolean determineElbowClose() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 10000) ;
  }
}

