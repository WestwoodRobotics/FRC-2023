package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;

public class ArmPositions extends CommandBase {

  Transport m_transport;
  IntakeModule m_intake;
  double shoulderPos;
  double elbowPos;
  double wristPos;
  double percentVolts;
  String currentPos;
  //boolean validPosition;
  Timer timer;
  double startTime;


  public ArmPositions(double shoulderPosition, double elbowPosition, double wristPosition, double percentVolts, Transport arm, IntakeModule intake) {
    
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

    this.shoulderPos = shoulderPosition;
    this.elbowPos = elbowPosition;
    this.wristPos = wristPosition;
    this.percentVolts = percentVolts;

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
    
    m_intake.setIntakePower(0.25);
  }

  @Override
  public void execute() {
    
    //SmartDashboard.putString("Current Position", currentPos);

    //Shoulder
    //Puts percent volts to shoulder until it reaches desired ticks
    if (!this.determineShoulderClose() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(percentVolts);
    } 
    else if (!this.determineShoulderClose() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-percentVolts);
    } 
    //decreases power when it is close to desired ticks to prevent rapidly going to 0 volts
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() < shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(percentVolts/2);
    }
    else if (!this.determineShoulderFinished() && (m_transport.getShoulderMotorPosition() > shoulderPos)) 
    {
      m_transport.setShoulderMotorPower(-(percentVolts/2));
    } 
    else if (this.determineShoulderFinished()) 
    {
      m_transport.setShoulderMotorPower(0);
    }

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

    //Wrist
    //Puts percent volts to wrist until it reaches desired ticks
    if (!this.determineWristClose() && (m_transport.getWristMotorPosition() < wristPos)) 
    {
      m_transport.setWristMotorPower(0.6);
    } 
    else if (!this.determineWristClose() && (m_transport.getWristMotorPosition() > wristPos)) 
    {
      m_transport.setWristMotorPower(-0.6);
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
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 0.75);
  }
  
  private boolean determineShoulderClose() {
    return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 0.75);
  }

  //Elbow
  private boolean determineElbowFinished() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 0.75);
  }

  private boolean determineElbowClose() {
    return (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 0.75);
  }

  //Wrist
  private boolean determineWristFinished() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 0.75);
  }

  private boolean determineWristClose() {
    return (Math.abs(m_transport.getWristMotorPosition() - wristPos) < 0.75);
  }
}

