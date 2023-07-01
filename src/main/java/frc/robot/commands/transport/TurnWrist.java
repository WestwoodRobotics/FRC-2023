package frc.robot.commands.transport;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.constants.TransportConstants;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

public class TurnWrist extends CommandBase {

  Transport transport;
  double wristPos;
  
  public TurnWrist(Transport t) {
    transport = t;
    addRequirements(t);
  }

  @Override
  public void initialize() {
    wristPos = TransportConstants.WRIST_FLIPPED_ROT;
  }

  @Override
  public void execute() {
    if (!this.determineWristClose() && (transport.getWristMotorPosition() < wristPos)) 
    {
      transport.setWristMotorPower(0.6);
    } 
    else if (!this.determineWristClose() && (transport.getWristMotorPosition() > wristPos)) 
    {
      transport.setWristMotorPower(-0.6);
    } 
    //decreases power when it is close to desired ticks to prevent rapidly going to 0 volts
    else if (!this.determineWristFinished() && (transport.getWristMotorPosition() < wristPos)) 
    {
      transport.setWristMotorPower(0.2);
    } 
    else if (!this.determineWristFinished() && (transport.getWristMotorPosition() > wristPos)) 
    {
      transport.setWristMotorPower(-0.2);
    } 
    else if (this.determineWristFinished()) 
    {
      transport.setWristMotorPower(0);
    }
  }
  
  private boolean determineWristFinished() {
    return (Math.abs(transport.getWristMotorPosition() - wristPos) < 0.5);
  }

  private boolean determineWristClose() {
    return (Math.abs(transport.getWristMotorPosition() - wristPos) < 4);
  }
}
