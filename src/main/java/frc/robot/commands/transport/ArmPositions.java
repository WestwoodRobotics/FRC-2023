package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;

public class ArmPositions extends CommandBase {

  Transport m_transport;
  double shoulderPos;
  double elbowPos;
  double wristPos;
  double percentVolts;
  String currentPos;
  boolean validPosition;


  public ArmPositions(String newPos, double shoulderPosition, double elbowPosition, double wristPosition, double percentVolts, Transport arm) {
    currentPos = arm.getPosition();
    if ((currentPos.equals("HIGH") || currentPos.equals("MID")) && (!newPos.equals("START"))) {
      validPosition = false;
    } else {
      validPosition = true;
    }
    
    this.shoulderPos = shoulderPosition;
    this.elbowPos = elbowPosition;
    this.wristPos = wristPosition;
    this.percentVolts = percentVolts;

    m_transport = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    System.out.println(m_transport.getShoulderMotorPosition());
    if (!this.determineShoulderClose() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(percentVolts);
    } 
    else if (!this.determineShoulderClose() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-percentVolts);
    } 
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(percentVolts/2);
    }
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-percentVolts/2);
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


    if (!this.determineWristClose() && (m_transport.getWristMotorPosition() < wristPos)) 
    {
      m_transport.setWristMotorPower(0.8);
    } 
    else if (!this.determineWristClose() && (m_transport.getWristMotorPosition() > wristPos)) 
    {
      m_transport.setWristMotorPower(-0.8);
    } 
    else if (!this.determineWristFinished() && (m_transport.getWristMotorPosition() < wristPos)) 
    {
      m_transport.setWristMotorPower(0.3);
    } 
    else if (!this.determineWristFinished() && (m_transport.getWristMotorPosition() > wristPos)) 
    {
      m_transport.setWristMotorPower(-0.3);
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
  }

  @Override
  public boolean isFinished() {
    return (determineShoulderFinished() && determineElbowFinished()) || !validPosition;
  }

  private boolean determineShoulderFinished() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 1250) ;
  }
  
  private boolean determineShoulderClose() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 10000) ;
  }

  private boolean determineElbowFinished() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 1000) ;
  }

  private boolean determineElbowClose() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 5000) ;
  }

  private boolean determineWristFinished() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 1000) ;
  }

  private boolean determineWristClose() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 5000) ;
  }
}

