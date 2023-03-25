package frc.robot.commands.swerve;

import java.time.Instant;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.subsystems.swerve.Gyro;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.transport.Transport;

public class AutoBalance extends CommandBase
{
    private SwerveDrive m_swerveDrive;
    private Gyro gyro; 

    public AutoBalance(SwerveDrive swerve, Gyro gyro)
    {
        this.gyro = gyro; 
        m_swerveDrive = swerve;
        addRequirements(swerve, gyro);
    }

    @Override
    public void initialize()
    {
        m_swerveDrive.setForwardTurn();
    }

    @Override
    public void execute()
    {
        //m_swerveDrive.drive(0, -1, 0, false);
        if( this.isBalance()){
            m_swerveDrive.drive(0, 0, 0, false); 
        }
        else if (this.isCloseTiltedBackward()){
            m_swerveDrive.drive(0, -0.4, 0, false); 
        }
        else if (this.isCloseTiltedForward()){
            m_swerveDrive.drive(0, 0.5, 0, false); 
        }
        else if (this.isTiltedBackward()){
            m_swerveDrive.drive(0, -0.4, 0, false); 
        }
        else if (this.isTiltedForward()){
            m_swerveDrive.drive(0, 0.2, 0, false); 
        }
        SmartDashboard.putNumber("gyro tilt", gyro.getRoll());
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

    public boolean isBalance(){
        return (Math.abs(gyro.getRoll()) <= 3); 
    }

    public boolean isCloseTiltedForward(){
        return (gyro.getRoll() > 10);
    }

    public boolean isTiltedForward(){
        return (gyro.getRoll() > 3);
    }

    public boolean isCloseTiltedBackward(){
        return (gyro.getRoll() < -10);
    }

    public boolean isTiltedBackward(){
        return (gyro.getRoll() < -3);
    }
}
