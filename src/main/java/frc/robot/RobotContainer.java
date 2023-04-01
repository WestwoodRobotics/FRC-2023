// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.List;
import java.util.function.Consumer;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.PortConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.commands.intake.UseIntake;
import frc.robot.commands.intake.slowOuttake;
import frc.robot.commands.swerve.AutoBalance;
import frc.robot.commands.swerve.DriveConstantControlCommand;
import frc.robot.commands.swerve.FeedforwardTest;
import frc.robot.commands.transport.ArmPositions;
import frc.robot.commands.transport.ArmPositionsV3;
import frc.robot.commands.transport.TurnWrist;
import frc.robot.commands.transport.UpdateWrist;
import frc.robot.commands.transport.ManualArm;
import frc.robot.commands.TimeAutonCommand;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.Gyro;
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
  private final JoystickButton xButton = new JoystickButton(primaryController, 3);
  private final JoystickButton rightBumper = new JoystickButton(primaryController, XboxController.Button.kRightBumper.value);
  private final JoystickButton leftBumper = new JoystickButton(primaryController, XboxController.Button.kLeftBumper.value);
  private final JoystickButton leftStickButton = new JoystickButton(primaryController, XboxController.Button.kLeftStick.value);
  private final JoystickButton rightStickButton = new JoystickButton(primaryController, XboxController.Button.kRightStick.value);
  private final JoystickButton startButton = new JoystickButton(primaryController, XboxController.Button.kStart.value);
  private final POVButton dPadUp = new POVButton(primaryController, 0);
  private final POVButton dPadRight = new POVButton(primaryController, 90);
  private final POVButton dPadDown = new POVButton(primaryController, 180);
  private final POVButton dPadLeft = new POVButton(primaryController, 270);

  private final POVButton secondDPadRight = new POVButton(secondaryController, 90);
  private final POVButton secondDPadLeft = new POVButton(secondaryController, 270);


  private final JoystickButton secondBButton = new JoystickButton(secondaryController, XboxController.Button.kB.value);
  private final JoystickButton secondXButton = new JoystickButton(secondaryController, XboxController.Button.kX.value);

  // The robot's subsystems and commands are defined here...
  private final SwerveDrive SwerveDriveSystem = new SwerveDrive();
  //private final SwerveModule swerveMod = SwerveDriveSystem.getModule(0);
  private final Transport transport = new Transport();
  private final IntakeModule intake = new IntakeModule();
  private final Gyro gyro = new Gyro();



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
    transport.setDefaultCommand(new ManualArm(primaryController, secondaryController, transport));
    intake.setDefaultCommand(new UseIntake(primaryController, secondaryController, intake));
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */

  private void configureButtonBindings() {
    //NEW POSITIONAS METHOD
    yButton.onTrue(new ArmPositionsV3("VERTICAL", transport, intake)
          .andThen(new ArmPositionsV3("HIGH", transport, intake)));
    //xButton.onTrue(new ArmPositionsV3("SHELF", transport, intake));
    aButton.onTrue(new ArmPositionsV3("GROUND", transport, intake));
    rightBumper.onTrue(new ArmPositionsV3("START", transport, intake));
    leftBumper.onTrue(new InstantCommand((() -> intake.incrementMode()))
              .andThen(new UpdateWrist(transport, intake)));
              //.andThen(new ArmPositionsV3("SAME", transport, intake)));

    /*
    yButton.onTrue(new ArmPositions(TransportConstants.VERTICAL_SHOULDER_ROT, TransportConstants.VERTICAL_ELBOW_ROT, TransportConstants.WRIST_START_ROT, transport, intake)
          .andThen(new ArmPositions(TransportConstants.HIGH_SHOULDER_ROT, TransportConstants.HIGH_ELBOW_ROT, TransportConstants.WRIST_START_ROT, transport, intake)));
    bButton.onTrue(new ArmPositions(TransportConstants.DROP_SHOULDER_ROT, TransportConstants.DROP_ELBOW_ROT, TransportConstants.WRIST_FLIPPED_ROT, transport, intake));
    xButton.onTrue(new ArmPositions(TransportConstants.SHELF_SHOULDER_ROT, TransportConstants.SHELF_ELBOW_ROT, TransportConstants.WRIST_FLIPPED_ROT, transport, intake));
    aButton.onTrue(new ArmPositions(TransportConstants.GROUND_SHOULDER_ROT, TransportConstants.GROUND_ELBOW_ROT, TransportConstants.WRIST_HALF_ROT, transport, intake));
    rightBumper.onTrue(new ArmPositions(TransportConstants.START_SHOULDER_ROT, TransportConstants.START_ELBOW_ROT, TransportConstants.WRIST_START_ROT, transport, intake));
    leftBumper.onTrue(new InstantCommand((() -> intake.incrementMode())));
    */
    dPadUp.whileTrue(new InstantCommand(() -> transport.setShoulderMotorPower(-0.15)));
    dPadDown.whileTrue(new InstantCommand(() -> transport.setShoulderMotorPower(0.1)));


    dPadRight.whileTrue(new InstantCommand(() -> SwerveDriveSystem.drive(-0.5, 0, 0, false)));
    dPadLeft.whileTrue(new InstantCommand(() -> SwerveDriveSystem.drive(0.5, 0, 0, false)));


    secondDPadLeft.onTrue(new TurnWrist(transport, -TransportConstants.WRIST_HALF_ROT));
    secondDPadRight.onTrue(new TurnWrist(transport, TransportConstants.WRIST_HALF_ROT));

    //secondDPadLeft.onTrue(new)

    //leftStickButton.onTrue(new InstantCommand(SwerveDriveSystem::resetGyro));
    //rightStickButton.onTrue(new InstantCommand(SwerveDriveSystem::zeroDrive));
    startButton.onTrue(new InstantCommand(SwerveDriveSystem::resetGyro));
    secondBButton.onTrue(new InstantCommand(transport::zeroTransportEncoders));
    secondXButton.onTrue(new InstantCommand(SwerveDriveSystem::resetGyro));

    // The following code is for the primary controller
    WrapperCommand resetMotorEncoderCommand = new InstantCommand(SwerveDriveSystem::resetAllEncoders).ignoringDisable(true);
    resetMotorEncoderCommand.setName("Recalibrate SwerveDrive Motor Encoder Positions");

    SmartDashboard.putData("Reset SwerveDrive Motor Encoders:", resetMotorEncoderCommand);

    // Returns positive values if the wheel turned clockwise from its starting position. (Starting position is the wheel's front facing the front of the robot)
  }


  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */

   /*TrajectoryConfig trajConfig = new TrajectoryConfig(AutoConstants.maxVelocity, AutoConstants.maxAcceleration)
   .setKinematics(SwerveConstants.swerveDriveKinematics);


 Trajectory traj = TrajectoryGenerator.generateTrajectory
 (
   new Pose2d(0, 0, new Rotation2d(0)),

   List.of
   (
     new Translation2d(0, -.5)
   ),

   new Pose2d(0, -1, Rotation2d.fromDegrees(0)),
   trajConfig
 );


 PIDController xController = new PIDController(AutoConstants.PID.kPControllerX, 0, AutoConstants.PID.kDControllerX),
               yController = new PIDController(AutoConstants.PID.kPControllerY, 0, AutoConstants.PID.kDControllerY);
 ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.PID.kPControllerTheta, 0, AutoConstants.PID.kDControllerTheta, AutoConstants.thetaControllerConstraints);
 thetaController.enableContinuousInput(-Math.PI, Math.PI);


 SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand
 (
   traj,
   SwerveDriveSystem::getPoseMeters,
   SwerveConstants.swerveDriveKinematics,
   xController,
   yController,
   thetaController,
   SwerveDriveSystem::setModuleStatesDirectly,
   SwerveDriveSystem
 );*/

  public Command getChargeStationAuto() {
    // An ExampleCommand will run in autonomous
    //ngl this is just copied but we can try to make changes to it to make it work.

    //put any other commands to do during auton in here
    //SmartDashboard.putBoolean("Auton entered", true);
    return new SequentialCommandGroup
    (
      //Score High + Autobalance
      //new InstantCommand(() -> SwerveDriveSystem.resetPose(traj.getInitialPose())), // Tell it that its initial pose is where it is
      new InstantCommand(() -> SwerveDriveSystem.setForwardTurn()),
      new ArmPositions(TransportConstants.VERTICAL_SHOULDER_ROT, TransportConstants.VERTICAL_ELBOW_ROT, TransportConstants.WRIST_START_ROT, transport, intake),
      new ArmPositions(130, TransportConstants.HIGH_ELBOW_ROT, TransportConstants.WRIST_START_ROT, transport, intake),
      new slowOuttake(intake),
      new ParallelCommandGroup(
            new ArmPositions(TransportConstants.START_SHOULDER_ROT, TransportConstants.START_ELBOW_ROT, TransportConstants.WRIST_START_ROT, transport, intake),
            new TimeAutonCommand(SwerveDriveSystem, 1.25, 3)),
      new TimeAutonCommand(SwerveDriveSystem, -1.25, 1.5),
      new AutoBalance(SwerveDriveSystem, gyro) // Go through the motions
    );
  }

  public void teleopTimer() {
    timer.reset();
    timer.start();
  }

  public void periodic() {
    SmartDashboard.putNumber("Timer:", 135 - timer.get());
    //SmartDashboard.putNumber("Shoulder Ticks", transport.getShoulderMotorPosition());
    //SmartDashboard.putNumber("Elbow Ticks", transport.getElbowMotorPosition());
    //SmartDashboard.putNumber("Wrist Ticks", transport.getWristMotorPosition());
    double shoulderPos = transport.getShoulderMotorPosition();
    double elbowPos = transport.getElbowMotorPosition();

  }

  public void disabledInit() {
    System.out.println("Saving encoder offsets");
    SwerveDriveSystem.saveEncoderOffsets();
  }
}
