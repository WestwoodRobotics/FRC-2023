package frc.robot.commands.TransportCommands;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;

public class ArmStopCommand extends CommandBase{
    private final Transport m_arm;

    public ArmStopCommand(Transport m_arm) {
        this.m_arm = m_arm;

    }

    // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println(getName());
    m_arm.setArmMotorPower(0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }   

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
    
}
