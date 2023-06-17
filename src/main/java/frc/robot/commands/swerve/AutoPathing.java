package frc.robot.commands.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Gyro;
import frc.robot.subsystems.swerve.SwerveDrive;

public class AutoPathing extends CommandBase
{
    private SwerveDrive m_swerveDrive;
    private Trajectory m_trajectory;
    private Timer time;

    public AutoPathing(SwerveDrive swerve, Trajectory trajectory)
    {
        m_swerveDrive = swerve;
        m_trajectory = trajectory;
        addRequirements(swerve);
        time = new Timer();
    }

    @Override
    public void initialize()
    {
        m_swerveDrive.setForwardTurn();
        time.reset();
        time.start();
        m_swerveDrive.resetPose(new Pose2d(0, 0, new Rotation2d(0)));
    }

    @Override
    public void execute()
    {
        Trajectory.State goal = m_trajectory.sample(time.get());
        Pose2d current = m_swerveDrive.getPoseMeters();
        ChassisSpeeds speeds = m_swerveDrive.getHolonomicDriveController().calculate(
            current,
            goal,
            new Rotation2d(0.0)
        );
        m_swerveDrive.drive(
            speeds.vxMetersPerSecond,
            speeds.vyMetersPerSecond, 
            speeds.omegaRadiansPerSecond, 
            true
        );
        SmartDashboard.putNumber("time",time.get());
        SmartDashboard.putNumber("goal_x", goal.poseMeters.getRotation().getRadians());
        SmartDashboard.putNumber("goal_y", goal.poseMeters.getY());
        SmartDashboard.putNumber("pose_x", current.getRotation().getRadians());
        SmartDashboard.putNumber("pose_y", current.getY());
        SmartDashboard.putNumber("speed_x", speeds.omegaRadiansPerSecond);
        SmartDashboard.putNumber("speed_y", speeds.vyMetersPerSecond);
    }

    @Override
    public boolean isFinished()
    {
        return false;
    }

    @Override
    public void end(boolean interrupted)
    {
        m_swerveDrive.drive(0, 0, 0, false);
        m_swerveDrive.zeroDrive();
        m_swerveDrive.zeroTurn();
    }
}
