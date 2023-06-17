package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

public class ManualArm extends CommandBase {

  Transport m_transport;
  XboxController primaryController;
  XboxController secondaryController;


  public ManualArm(XboxController primaryController, XboxController secondaryController, Transport arm) {
    this.primaryController = primaryController;
    this.secondaryController = secondaryController;
    

    m_transport = arm;
    addRequirements(arm);
  }

  @Override
  public void execute() {
    // todo: check to make sure these are all accurate, and perhaps move values to constants
    if(primaryController.getPOV() == 0) {
      m_transport.setShoulderMotorPower(-0.3);
    } 
    else if (primaryController.getPOV() == 180) {
      m_transport.setShoulderMotorPower(0.3);
    } else {
      m_transport.setShoulderMotorPower(
        -0.5 * Conversions.deadZoneSquare(secondaryController.getLeftY(), 0.1));
    }

    m_transport.setElbowMotorPower(
      0.5 * Conversions.deadZoneSquare(secondaryController.getRightY(), 0.1));

    m_transport.setWristMotorPower(
      -0.5 * Conversions.deadZoneSquare(secondaryController.getRightX(), 0.2));

    SmartDashboard.putNumber("shoulder ticks", m_transport.getShoulderMotorPosition());
    SmartDashboard.putNumber("elbow ticks", m_transport.getElbowMotorPosition());
    SmartDashboard.putNumber("wrist Ticks", m_transport.getWristMotorPosition());
    SmartDashboard.putNumber("shoulder abs", m_transport.getShoulderAbsPosition());
  }
}

