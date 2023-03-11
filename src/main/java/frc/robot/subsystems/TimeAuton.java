package frc.robot.subsystems;

import java.time.Instant;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.transport.Transport;

public class TimeAuton 
{
    private static SwerveDrive m_swerveDrive;

    private static Command sequence = new InstantCommand(() -> m_swerveDrive.drive(0, 1, 0,false));

    public TimeAuton()
    {}

    public static Command getTimeAuton(SwerveDrive swerve)
    {
        m_swerveDrive = swerve;
        sequence = sequence.andThen(new WaitCommand(2));
        return sequence;
    }
}
