// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableBuilderImpl;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.swerve.Gyro;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command autonomousCommand;

  private RobotContainer robotContainer;
  private Gyro gyro = new Gyro();
  private UsbCamera camera;

  private static final String chargeStationAuto = "Charging Station Auton";
  private static final String shortSideAuto = "Park Auton Short Side";
  private static final String longSideAuto = "Park Auton Long Side";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    camera = CameraServer.startAutomaticCapture("Camera", 0);
    camera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
    robotContainer = new RobotContainer();

    m_chooser.setDefaultOption("Charge Station Auton", chargeStationAuto);
    m_chooser.addOption("Park Auton Short Side", shortSideAuto);
    m_chooser.addOption("Park Auton Long Side", longSideAuto);

    SmartDashboard.putData("Auto choices", m_chooser);

  }

  /**
   * This function is called every robot packet, no matter the mode. Use this for items like
   * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /**
   * This function is called once each time the robot enters Disabled mode.
   */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /**
   * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    /*autnomousCommand = robotContainer.getAutonomousCommand();
    
    autonomousCommand.schedule();o*/
    m_autoSelected = m_chooser.getSelected();
    SmartDashboard.putString("Auto selected: ", m_autoSelected);

    switch (m_autoSelected){
      case shortSideAuto:
        break;
      case longSideAuto:
        break;
      case chargeStationAuto:
        default:
          autonomousCommand = robotContainer.getChargeStationAuto();
          break;
    }
    autonomousCommand.schedule();
  }

  /**
   * This function is called periodically during autonomous.
   */
  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putNumber("gyro roll teleop", gyro.getRoll());
    
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (autonomousCommand != null) {
      autonomousCommand.cancel();
    }
    

  }

  /**
   * This function is called periodically during operator control.
   */
  @Override
  public void teleopPeriodic() 
  {
    CameraServer.getServer().setSource(camera);
    SmartDashboard.putNumber("gyro roll teleop", gyro.getRoll());
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /**
   * This function is called periodically during test mode.
   */
  @Override
  public void testPeriodic() {}

  public RobotContainer geContainer() {
    return robotContainer;
  }
}
