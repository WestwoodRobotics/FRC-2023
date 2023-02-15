// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.ExampleSubsystem;
import frc.robot.subsystems.Intake.Intake;


/** An example command that uses an example subsystem. */
public class IntakeOpenCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private final Timer tim = new Timer();
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeOpenCommand(Intake m_intake) {
    this.m_intake = m_intake;


    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // m_intake.setIntakeVelocity(100);
    m_intake.setIntakePower(.1);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
 

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // m_intake.setIntakeVelocity(0);
    m_intake.setIntakePower(0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
