// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;


/** An example command that uses an example subsystem. */
public class HoldIntakeRoller extends CommandBase {
  private final Intake m_intake;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public HoldIntakeRoller(Intake intake) {
    m_intake = intake;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_intake.setIntakePosition(m_intake.getPosition());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;

  }
}
