package frc.robot.commands.transport;

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
    m_transport.setShoulderMotorPosition(shoulderPos);
    m_transport.setElbowMotorPosition(elbowPos);
  }

  @Override
  public boolean isFinished() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 1000) && (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 1000);
  }
}

