// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeCommands.IntakeOpenCommand;
import frc.robot.commands.IntakeCommands.SetIntakePositionCommand;
import frc.robot.commands.IntakeCommands.IntakeCloseCommand;
import frc.robot.subsystems.Intake.*;
import frc.robot.Constants.IntakeConstants;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final Intake m_intake = new Intake();

  private final XboxController mainController = new XboxController(/*fake number */0);


  private final JoystickButton lBumper = new JoystickButton(mainController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rBumper = new JoystickButton(mainController, XboxController.Button.kRightBumper.value);
  private final JoystickButton buttonA = new JoystickButton(mainController, XboxController.Button.kA.value);
  private final JoystickButton buttonB = new JoystickButton(mainController, XboxController.Button.kB.value);
  
  //private final IntakeOpenCommand instanceOfIntakeOpenCommand = new IntakeOpenCommand(m_intake);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    
    // Configure the button bindings
    configureButtonBindings();
    m_intake.setDefaultCommand(Commands.run(() -> {System.out.println(m_intake.getPosition());}, m_intake));
    
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    lBumper.whileTrue(new IntakeOpenCommand(m_intake));
    rBumper.whileTrue(new IntakeCloseCommand(m_intake));
    // buttonA.onTrue(new SetIntakePositionCommand(m_intake, (m_intake.initialPosition + Constants.IntakeConstants.kCLOSE_OPEN_DIFFERENCE)));
    // buttonB.onTrue(new SetIntakePositionCommand(m_intake, m_intake.initialPosition));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return null;
  }
}
