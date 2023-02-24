// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Wrist;


/** An example command that uses an example subsystem. */
public class RotateWrist extends CommandBase {
  private final Wrist wrist;
  private final double position;
  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */
  public RotateWrist(Wrist wrist, double position) {
    this.wrist = wrist;
    this.position = position;
    

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(wrist);
  }

  

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    wrist.setWristPosition(position + wrist.initialPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(wrist.getPosition() - position) < 50) { 
      return true;
    }
    return false;
  }
}
