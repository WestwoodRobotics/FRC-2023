package frc.robot.commands.swerve;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.Gyro;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.vision.LED;

public class AutoBalance extends CommandBase
{
    private SwerveDrive m_swerveDrive;
    private Gyro gyro;
    private Timer time;
    private LED led;

    boolean wasBalanced = false;

    public AutoBalance(SwerveDrive swerve, Gyro gyro, LED led)
    {
        this.gyro = gyro;
        m_swerveDrive = swerve;
        this.led = led;
        addRequirements(swerve, gyro);
        time = new Timer();
    }

    @Override
    public void initialize()
    {
        m_swerveDrive.setForwardTurn();
    }

    @Override
    public void execute()
    {

        //m_swerveDrive.drive(0, -1,0, false);
        if( this.isBalance()){
            m_swerveDrive.drive(0, 0, 0, false);

        }
        else if (this.isCloseTiltedBackward()){
            m_swerveDrive.drive(0, -0.4, 0, false);
        }
        else if (this.isCloseTiltedForward()){
            m_swerveDrive.drive(0, 0.4, 0, false);
        }
        else if (this.isTiltedBackward()){
            m_swerveDrive.drive(0, -0.275, 0, false);
        }
        else if (this.isTiltedForward()){
            m_swerveDrive.drive(0, 0.275, 0, false);
        }

        if ( !wasBalanced && this.isBalance() ) {
            wasBalanced = true;
            time.reset();
            time.start();
        }
        else if (!this.isBalance()) {
            wasBalanced = false;
            led.setOrange();
            time.stop();
        }
        else if ( wasBalanced && (time.get() > 0.5)) {
            led.setGreen();
        }

    }

    @Override
    public boolean isFinished()
    {
        return (wasBalanced && (time.get() > 0.5));
    }

    @Override
    public void end(boolean interrupted)
    {
        m_swerveDrive.drive(0, 0, 0, false);
        m_swerveDrive.zeroDrive();
        m_swerveDrive.zeroTurn();
    }

    public boolean isBalance(){
        return (Math.abs(gyro.getRoll()) <= 5);
    }

    public boolean isCloseTiltedForward(){
        return (gyro.getRoll() > 10);
    }

    public boolean isTiltedForward(){
        return (gyro.getRoll() > 5);
    }

    public boolean isCloseTiltedBackward(){
        return (gyro.getRoll() < -10);
    }

    public boolean isTiltedBackward(){
        return (gyro.getRoll() < -5);
    }


}
