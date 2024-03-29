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
  

  private static final String conechargeStationAuto = "Cone Charging Station Auton";
  private static final String coneshortSideAuto = "Cone Park Auton Short Side";
  private static final String conelongSideAuto = "Cone Park Auton Long Side";
  private static final String conehighOnlyAuto = "Cone High Cone Only";
  private static final String cubechargeStationAuto = "Cube Charging Station Auton";
  private static final String cubeshortSideAuto = "Cube Park Auton Short Side";
  private static final String cubelongSideAuto = "Cube Park Auton Long Side";
  private static final String cubehighOnlyAuto = "Cube High Cone Only";
  private static final String testPathAuto = "Test Path Auto";
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

    m_chooser.setDefaultOption("Cone Charge Station Auto", conechargeStationAuto);
    m_chooser.addOption("Cone Park Auto Short Side", coneshortSideAuto);
    m_chooser.addOption("Cone Park Auto Long Side", conelongSideAuto);
    m_chooser.addOption("Cone High Only", conehighOnlyAuto);
    m_chooser.setDefaultOption("Cube Charge Station Auto", cubechargeStationAuto);
    m_chooser.addOption("Cube Park Auto Short Side", cubeshortSideAuto);
    m_chooser.addOption("Cube Park Auto Long Side", cubelongSideAuto);
    m_chooser.addOption("Cube High Only", cubehighOnlyAuto);
    m_chooser.addOption("Test Path Auto", testPathAuto);

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
    this.geContainer().setCubeMode(this.geContainer().getIntake().getCubeMode());
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
      case testPathAuto:
        autonomousCommand = robotContainer.getTestPathAuto();
        break;
      case coneshortSideAuto:
        autonomousCommand = robotContainer.getConeShortSideAuto();
        break;
      case conelongSideAuto:
        autonomousCommand = robotContainer.getConeLongSideAuto();
        break;
      case conehighOnlyAuto:
        autonomousCommand = robotContainer.getConeHighOnlyAuto();
      case cubeshortSideAuto:
        autonomousCommand = robotContainer.getCubeShortSideAuto();
        break;
      case cubelongSideAuto:
        autonomousCommand = robotContainer.getCubeLongSideAuto();
        break;
      case cubehighOnlyAuto:
        autonomousCommand = robotContainer.getCubeHighOnlyAuto();
        break;
      case cubechargeStationAuto:
        autonomousCommand = robotContainer.getCubeChargeStationAuto();
        break;
      case conechargeStationAuto:
        default:
          autonomousCommand = robotContainer.getConeChargeStationAuto();
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
