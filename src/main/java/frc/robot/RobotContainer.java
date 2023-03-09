// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.constants.PortConstants;
import frc.robot.commands.intake.UseIntake;
import frc.robot.commands.swerve.DriveConstantControlCommand;
import frc.robot.commands.swerve.FeedforwardTest;
import frc.robot.commands.transport.ArmPositions;
import frc.robot.commands.transport.ManualArm;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;
import frc.robot.subsystems.transport.Transport;
import frc.robot.constants.TransportConstants;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  // The XBox Controllers are being initialized here
  private final XboxController primaryController = new XboxController(PortConstants.primaryControllerPort);
  private final XboxController secondaryController = new XboxController(PortConstants.secondaryControllerPort);

  private final JoystickButton yButton = new JoystickButton(primaryController, XboxController.Button.kY.value);
  private final JoystickButton aButton = new JoystickButton(primaryController, XboxController.Button.kA.value);
  private final JoystickButton bButton = new JoystickButton(primaryController, XboxController.Button.kB.value);
  private final JoystickButton xButton = new JoystickButton(primaryController, XboxController.Button.kX.value);
  private final JoystickButton rightBumper = new JoystickButton(primaryController, XboxController.Button.kRightBumper.value);
  private final JoystickButton leftBumper = new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton leftStickButton = new JoystickButton(primaryController, XboxController.Button.kLeftStick.value);
  private final JoystickButton rightStickButton = new JoystickButton(primaryController, XboxController.Button.kRightStick.value);
  private final JoystickButton secondBButton = new JoystickButton(secondaryController, XboxController.Button.kB.value);
  // The robot's subsystems and commands are defined here...
  private final SwerveDrive SwerveDriveSystem = new SwerveDrive();
  //private final SwerveModule swerveMod = SwerveDriveSystem.getModule(0);
  private final Transport transport = new Transport();
  private final IntakeModule intake = new IntakeModule();
  


  //private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation


  // Instantiating the timer
  private final Timer timer = new Timer();


  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem); <--- This is an example "command" implementation


  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
    setDefaultCommands();
  }

  private void setDefaultCommands() {
    //swerveMod.setDefaultCommand(new FeedforwardTest(swerveMod, 1));
    SwerveDriveSystem.setDefaultCommand(new DriveConstantControlCommand(SwerveDriveSystem, primaryController));
    transport.setDefaultCommand(new ManualArm(secondaryController, transport));
    intake.setDefaultCommand(new UseIntake(primaryController, secondaryController, intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    yButton.onTrue(new ArmPositions(TransportConstants.VERTICAL_SHOULDER_TICKS, TransportConstants.VERTICAL_ELBOW_TICKS, transport).andThen(new ArmPositions(TransportConstants.HIGH_SHOULDER_TICKS, TransportConstants.HIGH_ELBOW_TICKS, transport)));
    bButton.onTrue(new ArmPositions(TransportConstants.VERTICAL_SHOULDER_TICKS, TransportConstants.VERTICAL_ELBOW_TICKS, transport).andThen(new ArmPositions(TransportConstants.MID_SHOULDER_TICKS, TransportConstants.MID_ELBOW_TICKS, transport)));
    xButton.onTrue(new ArmPositions(TransportConstants.SHELF_SHOULDER_TICKS, TransportConstants.SHELF_ELBOW_TICKS, transport));
    aButton.onTrue(new ArmPositions(TransportConstants.GROUND_SHOULDER_TICKS, TransportConstants.GROUND_ELBOW_TICKS, transport));
    rightBumper.onTrue(new ArmPositions(TransportConstants.START_SHOULDER_TICKS, TransportConstants.START_ELBOW_TICKS, transport));
    
    leftStickButton.onTrue(new InstantCommand(SwerveDriveSystem::resetGyro));
    rightStickButton.onTrue(new InstantCommand(SwerveDriveSystem::zeroDrive));
    secondBButton.onTrue(new InstantCommand(transport::zeroTransportEncoders));
    

    // The following code is for the primary controller
    WrapperCommand resetMotorEncoderCommand = new InstantCommand(SwerveDriveSystem::resetAllEncoders).ignoringDisable(true);
    resetMotorEncoderCommand.setName("Recalibrate SwerveDrive Motor Encoder Positions");

    WrapperCommand printAssumedCurrentWheelAngles = new InstantCommand(SwerveDriveSystem::printSteerAngles).ignoringDisable(true);
    printAssumedCurrentWheelAngles.setName("Print SwerveDrive Steer Motor Wheel Angles");

    WrapperCommand printAllAssumedTransportMotorRawEncoderTicks = new InstantCommand(transport::printAllMotorRawEncoderTicks).ignoringDisable(true);
    printAllAssumedTransportMotorRawEncoderTicks.setName("Print Trnsport Motor Raw Encoder Ticks");

    WrapperCommand printAllAssumedTransportMotorAngles = new InstantCommand(transport::printAllMotorCalculatedAngles).ignoringDisable(true);
    printAllAssumedTransportMotorAngles.setName("Print Transport Motor Computed Angles");


    //SmartDashboard.putData("Reset Motor Encoders:", resetMotorEncoderCommand);

    SmartDashboard.putData("Reset SwerveDrive Motor Encoders:", resetMotorEncoderCommand);

    // Returns positive values if the wheel turned clockwise from its starting position. (Starting position is the wheel's front facing the front of the robot)
    SmartDashboard.putData("Current Presumed SwerveDrive Steer Motor Angles:", printAssumedCurrentWheelAngles);

    SmartDashboard.putData("Current Transport Motor Raw Encoder Ticks:", printAllAssumedTransportMotorRawEncoderTicks);

    SmartDashboard.putData("Current Transport Motor Computed Angles:", printAllAssumedTransportMotorAngles);

    
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
    SmartDashboard.putNumber("Shoulder Ticks", transport.getShoulderMotorPosition());
    double shoulderPos = transport.getShoulderMotorPosition();
    double elbowPos = transport.getElbowMotorPosition();
  }

  public void disabledInit() {
    System.out.println("Saving encoder offsets");
    SwerveDriveSystem.saveEncoderOffsets();

  }
}
