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
  //private final XboxController secondaryController = new XboxController(PortConstants.XboxController2);

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive SwerveDriveSystem = new SwerveDrive();

  //Instantiating the timer
  private final Timer timer = new Timer();

  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation




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


    Sendable resetMotorEncoderCommand = new InstantCommand(() -> {
      System.out.println("Encoders reset!");
      SwerveDriveSystem.resetAllEncoders();
    });

    Sendable printAssumedCurrentWheelAngles = new InstantCommand(() -> {
      SwerveDriveSystem.printDriveTrainSteerMotorDegrees();
    });

    SmartDashboard.putData("Reset Motor Encoders:", resetMotorEncoderCommand);
    SmartDashboard.putData("Current Presumed Steer Motor Angles:", printAssumedCurrentWheelAngles); //Returns positive values if the wheel turned clockwise from it's starting position. (Starting position is the wheel's front facing the front of the robot)
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
