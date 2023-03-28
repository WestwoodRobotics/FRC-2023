package frc.robot.commands;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryGenerator.ControlVectorList;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.constants.AutoConstants;
import frc.robot.constants.DriveConstants;
import frc.robot.constants.SwerveConstants;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.vision.Vision;

public class VisionAlign extends CommandBase
{
    private SwerveDrive m_swerveDrive;
    private Transport m_transport;
    private Vision m_vision;
    private double startTime;
    private Timer timer;

    public VisionAlign(SwerveDrive swerve, Transport trans, Vision vision)
    {
        m_swerveDrive = swerve;
        m_transport = trans;
        m_vision = vision;

        addRequirements(vision, swerve, trans);
    }

    @Override
    public void initialize()
    {
        timer = new Timer();
        timer.reset();
        timer.start();
        startTime = timer.get();
    }

    @Override
    public void execute()
    {
        m_swerveDrive.drive(DriveConstants.maxSpeed * Math.sin(m_vision.getHorizontalDiff()), 0, 0,false);
    }

    @Override
    public boolean isFinished()
    {
        return ((timer.get() - startTime) >= 3) || (Math.abs(m_vision.getHorizontalDiff()) <= 3);
    }

    @Override
    public void end(boolean interrupted)
    {
        m_swerveDrive.zeroDrive();
        m_swerveDrive.zeroTurn();
    }

}
