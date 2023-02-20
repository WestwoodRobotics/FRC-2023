// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.InstantCommand;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.PortConstants;
import frc.robot.commands.SwerveDriveCommands.DriveConstantControlCommand;
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

  //Instantiating the timer
  private final Timer timer = new Timer();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation

  // The buttons on the Primary Xbox Controller are being initialized here
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

    //The following code is for the primary controller


    Sendable resetEncoderCommand = new InstantCommand(() -> {
      System.out.println("Encoders reset!");
      SwerveDriveSystem.resetAllEncoders();
    });

    Sendable printAssumedCurrentWheelAngles = new InstantCommand(() -> {
      SwerveDriveSystem.printDriveTrainSteerMotorDegrees();
    });

    SmartDashboard.putData("Reset Encoders:", resetEncoderCommand);
    SmartDashboard.putData("Current Presumed Steer Motor Angles:", printAssumedCurrentWheelAngles);



  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */

  public void teleopTimer() {
    timer.reset();
    timer.start();
  }

  public void periodic() {
    SmartDashboard.putNumber("Timer:", 135 - timer.get());
  }

  public void disabledInit() {
    System.out.println("Saving encoder offsets");
    SwerveDriveSystem.saveEncoderOffsets();
  }



}
