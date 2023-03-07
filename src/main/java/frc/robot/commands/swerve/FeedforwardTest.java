package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.swerve.DriveSpeed;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.swerve.SwerveModule;

import static frc.robot.Constants.DriveConstants.maxAngularSpeed;
import static frc.robot.Constants.DriveConstants.maxSpeed;


public class FeedforwardTest extends CommandBase
{
    private SwerveModule m_module;
    private double percentVolts;

    public FeedforwardTest(SwerveModule mod, double percent)
    {
        m_module = mod;
        percentVolts = percent;

        addRequirements(mod);
    }

    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() 
    {
        m_module.setPercentVoltage(percentVolts);
        SmartDashboard.putNumber("Input Percent", m_module.setPercentVoltage(percentVolts));
        
        SmartDashboard.putNumber("Output Speed", m_module.getVelocity(1));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // should never end in teleop
    return false;
  }
}
