package frc.robot.commands.transport;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.TransportConstants;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;

public class ArmPositionsV3 extends CommandBase
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
  private double updateBegin;
  private double updateEnd;
  private Constraints constraints;
  private TrapezoidProfile shoulderProfile;
  private State start;
  private State target;


  public ArmPositionsV3(String position, Transport arm, IntakeModule intake) {

    timer = new Timer();
    //mode = intake.getIntakeMode();
    this.position = position;
    m_transport = arm;
    m_intake = intake;
    constraints = new TrapezoidProfile.Constraints(1, 1);
    addRequirements(arm, intake);

    //m_transport.setPosition(newPos);
  }

  @Override
  public void initialize() {

    if(position.equals("UPDATE")) {
      position = m_transport.getPos();
    }

    m_transport.setPos(position);

    switch(this.position) {
      case "VERTICAL":
        shoulderPos = TransportConstants.VERTICAL_SHOULDER_ROT;
        elbowPos = TransportConstants.VERTICAL_ELBOW_ROT;
      case "HIGH":
        elbowPos = TransportConstants.HIGH_ELBOW_ROT;
        if (m_intake.getIntakeMode() != 1){
          shoulderPos = TransportConstants.HIGH_SHOULDER_ROT;
        }
        else {

          shoulderPos = TransportConstants.HIGH_CUBE_SHOULDER_ROT;
        }
        break;
      case "MIDDLE":
        shoulderPos = TransportConstants.SHELF_SHOULDER_ROT;
        elbowPos = TransportConstants.SHELF_ELBOW_ROT;

      case "GROUND":
        if (m_intake.getIntakeMode() == 0) {
          shoulderPos = TransportConstants.GROUND_CONE_SHOULDER_ROT;
          elbowPos = TransportConstants.GROUND_CONE_ELBOW_ROT;

        }
        else if (m_intake.getIntakeMode() == 1) {
          shoulderPos = TransportConstants.GROUND_CUBE_SHOULDER_ROT;
          elbowPos = TransportConstants.GROUND_CUBE_ELBOW_ROT;

        }
        break;
      case "START":
        shoulderPos = TransportConstants.START_SHOULDER_ROT;
        elbowPos = TransportConstants.START_ELBOW_ROT;

        break;
      default:
        shoulderPos = 0;
        elbowPos = 0;
        wristPos = 0;
    }

    //m_transport.setPos(position);
    SmartDashboard.putString("Transport Position", position);

    timer.reset();
    timer.start();
    startTime = timer.get();
    updateBegin = timer.get();

    m_intake.setIntakePower(m_intake.intakeInverted(m_intake.getIntakeMode()) * 0.1);

    start = new TrapezoidProfile.State(m_transport.getShoulderMotorPosition(), m_transport.getShoulderMotorVelocityRPS());

    shoulderProfile = new TrapezoidProfile(constraints, new TrapezoidProfile.State(shoulderPos, 0), start);
  }

  @Override
  public void execute() {
    /*
    updateEnd = timer.get();
    target = shoulderProfile.calculate(updateEnd - updateBegin);
    m_transport.setShoulderMotorPosition((float)(target.position), (float)(target.velocity / TransportConstants.shoulderMaxSpeed));
    updateBegin = timer.get();
    */

    m_transport.setShoulderMotorPosition(shoulderPos, 0);
    //ELbow
    //Puts percent volts to elbow until it reaches desired ticks
    if (!this.determineElbowClose() && (m_transport.getElbowMotorPosition() < elbowPos))
    {
      m_transport.setElbowMotorPower(1);
    }
    else if (!this.determineElbowClose() && (m_transport.getElbowMotorPosition() > elbowPos))
    {
      m_transport.setElbowMotorPower(-1);
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
    /*
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
   */
    //m_transport.setShoulderMotorPosition(shoulderPos);
    //m_transport.setElbowMotorPosition(elbowPos);
  }

  @Override
  public void end(boolean interrupted) {
    m_transport.setShoulderMotorPower(0);
    m_transport.setElbowMotorPower(0);
    //m_transport.setWristMotorPower(0);
  }

  @Override
  public boolean isFinished() {
    return (determineShoulderFinished() && determineElbowFinished()) || ((timer.get() - startTime) > 3);
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

}
