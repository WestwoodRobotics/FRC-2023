// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;

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

  // The XBox Controllers are being initialized here
  private final XboxController primaryController = new XboxController(PortConstants.XboxController1);
  private final XboxController secondaryController = new XboxController(PortConstants.XboxController2);
  // private final XboxController secondaryController = new XboxController(PortConstants.XboxController2);

  // The robot's subsystems and commands are defined here...
   private final SwerveDrive SwerveDriveSystem = new SwerveDrive();
  private final Transport transport = new Transport();
  private final IntakeModule intake  = new IntakeModule();


  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation


  // Instantiating the timer
  private final Timer timer = new Timer();


  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation


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
    transport.setDefaultCommand(new UseArm(secondaryController, transport));
    intake.setDefaultCommand(new UseIntake(secondaryController, intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
   
  private void configureButtonBindings() {

    // The following code is for the primary controller
    WrapperCommand resetMotorEncoderCommand = new InstantCommand(SwerveDriveSystem::resetAllEncoders).ignoringDisable(true);
    resetMotorEncoderCommand.setName("Recalibrate Motor Encoder positions");

    WrapperCommand printAssumedCurrentWheelAngles = new InstantCommand(SwerveDriveSystem::printSteerAngles).ignoringDisable(true);
    printAssumedCurrentWheelAngles.setName("Print Current Wheel Angles");

    SmartDashboard.putData("Reset Motor Encoders:", resetMotorEncoderCommand);

    // Returns positive values if the wheel turned clockwise from its starting position. (Starting position is the wheel's front facing the front of the robot)
    SmartDashboard.putData("Current Presumed Steer Motor Angles:", printAssumedCurrentWheelAngles);
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */

  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    DriveConstantControlCommand x = new DriveConstantControlCommand(SwerveDriveSystem, primaryController);
    //SwerveDriveSystem.setDefaultCommand(x);
    return x;
  }

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
