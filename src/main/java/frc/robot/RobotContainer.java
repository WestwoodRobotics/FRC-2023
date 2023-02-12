// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.HashMap;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
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

  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   */
  public Command getAutonomousCommand() {
    
    // This will load the file "Example Path Group.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
ArrayList<PathPlannerTrajectory> pathGroup1 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("New Path", new PathConstraints(4, 3));

// This will load the file "Example Path Group.path" and generate it with different path constraints for each segment
ArrayList<PathPlannerTrajectory> pathGroup2 = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup(
    "New Path", 
    new PathConstraints(4, 3), 
    new PathConstraints(2, 2), 
    new PathConstraints(3, 3));
    // This will load the file "FullAuto.path" and generate it with a max velocity of 4 m/s and a max acceleration of 3 m/s^2
// for every path in the group
ArrayList<PathPlannerTrajectory> pathGroup = (ArrayList<PathPlannerTrajectory>) PathPlanner.loadPathGroup("FullAuto", new PathConstraints(4, 3));

// This is just an example event map. It would be better to have a constant, global event map
// in your code that will be used by all path following commands.
HashMap<String, Command> eventMap = new HashMap<>();
eventMap.put("marker1", new PrintCommand("Passed marker 1"));
//eventMap.put("intakeDown", new IntakeDown())


// Create the AutoBuilder. This only needs to be created once when robot code starts, not every time you want to create an auto command. A good place to put this is in RobotContainer along with your subsystems.
SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
    SwerveDriveSystem::getPoseMeters, // Pose2d supplier
    SwerveDriveSystem::resetPose, // Pose2d consumer, used to reset odometry at the beginning of auto
    Constants.DriveConstants.kDriveKinematics, // SwerveDriveKinematics
    new PIDConstants(5.0, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
    new PIDConstants(0.5, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
    SwerveDriveSystem::setStates, // Module states consumer used to output to the drive subsystem
    eventMap,
    true, // Should the path be automatically mirrored depending on alliance color. Optional, defaults to true
    SwerveDriveSystem // The drive subsystem. Used to properly set the requirements of path following commands
    
);

return autoBuilder.fullAuto(pathGroup);

    }

  public void teleopTimer() {
    timer.reset();
    timer.start();
  }

    public void periodic() {
    SmartDashboard.putNumber("Timer:", 135 - timer.get());
  }

    public void disabledInit() {
    SwerveDriveSystem.saveEncoderOffsets();
  }

}
