package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.transport.Transport;

public class TimeAutonCommand extends CommandBase
{
    private SwerveDrive m_swerveDrive;
    private Timer timer;
    private double speed;
    private double time;

    public TimeAutonCommand(SwerveDrive swerve, double speed, double time)
    {
        this.speed = speed;
        this.time = time;
        m_swerveDrive = swerve;
        timer = new Timer();
        addRequirements(swerve);
    }

    @Override
    public void initialize()
    {
        m_swerveDrive.setForwardTurn();
        timer.reset();
        timer.start();
        //startTime = timer.get();
        //m_swerveDrive.drive(0, -1, 0, false);
        //new WaitCommand(3); //This might lead to problems

    }

    @Override
    public void execute()
    {
        m_swerveDrive.drive(-0.1, -speed, 0, false);
    }

    @Override
    public boolean isFinished()
    {
        //SmartDashboard.putNumber("timer", timer.get());
        //return (timer.get() >= 3.5);
        return (timer.get() >= time);
        //return true;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_swerveDrive.drive(0, 0, 0, false);
        m_swerveDrive.zeroDrive();
        m_swerveDrive.zeroTurn();
    }
}
