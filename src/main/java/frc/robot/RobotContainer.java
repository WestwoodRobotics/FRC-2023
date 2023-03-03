// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.TransportCommands.UseArm;
import frc.robot.commands.Intake.UseIntake;
import frc.robot.commands.SwerveDriveCommands.DriveConstantControlCommand;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.transport.Transport;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  //THe XBox Controllers are being initialized here
  private final XboxController primaryController = new XboxController(PortConstants.XboxController1);
  private final XboxController secondaryController = new XboxController(PortConstants.XboxController2);
  // The robot's subsystems and commands are defined here...
   private final SwerveDrive SwerveDriveSystem = new SwerveDrive();
  private final Transport transport = new Transport();
  private final IntakeModule intake  = new IntakeModule();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation

  // The buttons on the Primary Xbox Controller are being initialized here
  private final JoystickButton rBumper = new JoystickButton(primaryController, XboxController.Button.kRightBumper.value);
  private final JoystickButton lBumper = new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton yButton = new JoystickButton(primaryController, XboxController.Button.kY.value);
  private final JoystickButton xButton = new JoystickButton(primaryController, XboxController.Button.kX.value);
  private final JoystickButton bButton = new JoystickButton(primaryController, XboxController.Button.kB.value);
  private final JoystickButton aButton = new JoystickButton(primaryController, XboxController.Button.kA.value);

  //The buttons on the Secondary Xbox Controller are being initialized here
  private final JoystickButton startButton = new JoystickButton(primaryController, XboxController.Button.kStart.value);
  private final JoystickButton hangarYButton = new JoystickButton(secondaryController, XboxController.Button.kY.value);
  private final JoystickButton hangarXButton = new JoystickButton(secondaryController, XboxController.Button.kX.value);
  private final JoystickButton hangarBButton = new JoystickButton(secondaryController, XboxController.Button.kB.value);
  private final JoystickButton hangarAButton = new JoystickButton(secondaryController, XboxController.Button.kA.value);


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    //configureButtonBindings(); DON'T ACCEPT THIS DURING MERGE!!!!
    setDefaultCommands();
  }

  private void setDefaultCommands() {
    SwerveDriveSystem.setDefaultCommand(new DriveConstantControlCommand(SwerveDriveSystem, primaryController));
    transport.setDefaultCommand(new UseArm(primaryController, transport));
    intake.setDefaultCommand(new UseIntake(primaryController, intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
 


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    DriveConstantControlCommand x = new DriveConstantControlCommand(SwerveDriveSystem, primaryController);
    //SwerveDriveSystem.setDefaultCommand(x);
    return x;
  }
}
