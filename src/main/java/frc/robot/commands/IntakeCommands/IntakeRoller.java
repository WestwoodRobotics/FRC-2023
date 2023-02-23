// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;


/** An example command that uses an example subsystem. */
public class IntakeRoller extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Intake m_intake;
  private double wntPosition;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public IntakeRoller(Intake m_intake, double wntPosition) {
    this.m_intake = m_intake;
    this.wntPosition = wntPosition; // Intake motor sensor position value

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_intake);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_intake.setIntakePosition(m_intake.getPosition() + wntPosition);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_intake.setIntakeVelocity(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(m_intake.getPosition() - wntPosition) < 50) {
      return true;
    }
    return false;
  }
}
