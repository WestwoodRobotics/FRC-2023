// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.lang.ModuleLayer.Controller;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.IntakeCommands.*;
import frc.robot.commands.SwerveDriveCommands.DriveConstantControlCommand;
import frc.robot.subsystems.intake.*;
import frc.robot.Constants.IntakeConstants.*;
import frc.robot.subsystems.swerve.SwerveDrive;

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
  private final Intake intakeSystem = new Intake();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation

  // The buttons on the Primary Xbox Controller are being initialized here
  private final JoystickButton lBumper = new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton rBumper = new JoystickButton(primaryController, XboxController.Button.kRightBumper.value);
  private final JoystickButton Bumper = new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value);
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
    configureButtonBindings();
    setDefaultCommands();
    intakeSystem.setDefaultCommand(new HoldIntakeRoller(intakeSystem));
    // -264 open
    // 155 closed
  }

  private void setDefaultCommands() {
    SwerveDriveSystem.setDefaultCommand(new DriveConstantControlCommand(SwerveDriveSystem, primaryController));

  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    lBumper.whileTrue(new IntakeInRoller(intakeSystem));
    rBumper.whileTrue(new IntakeOutRoller(intakeSystem));
    aButton.onTrue(new SetIntakePositionCommand(intakeSystem, (intakeSystem.initialPosition - 866/*Constants.IntakeConstants.kOPEN_INTAKE))*/)));
    bButton.onTrue(new SetIntakePositionCommand(intakeSystem, intakeSystem.initialPosition  /*+Constants.IntakeConstants.kCLOSED_INTAKE_CUBE)*/));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    DriveConstantControlCommand x = new DriveConstantControlCommand(SwerveDriveSystem, primaryController);
    SwerveDriveSystem.setDefaultCommand(x);
    return x;
  }
}
