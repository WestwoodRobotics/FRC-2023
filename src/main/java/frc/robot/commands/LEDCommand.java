package frc.robot.commands;

import java.sql.Time;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.vision.LED;

public class LEDCommand extends CommandBase
{
    private LED m_LED;
    private boolean cycleSpot;
    private boolean cubeMode;
    private Timer timer;
    private double startTime;

    public LEDCommand(LED led, boolean cube)
    {
        m_LED = led;
        cycleSpot = true;
        cubeMode = cube;
        addRequirements(led);
        timer = new Timer();
    }

    @Override
    public void initialize()
    {
        for (int i = 0; i < m_LED.getBuffer().getLength(); i++)
        {
            if (cubeMode)
            {
                if (cycleSpot)
                {
                    m_LED.setPurple(i);
                }
                else
                {
                    m_LED.lightOff(i);
                }
            }
            else
            {
                if (cycleSpot)
                {
                    m_LED.setYellow(i);
                }
                else
                {
                    m_LED.lightOff(i);
                }
            }
            cycleSpot = !cycleSpot;
        }
        m_LED.setData();
        timer.reset();
        timer.start();
        startTime = timer.get();
    }

    @Override
    public void execute()
    {
        if (cubeMode)
            m_LED.setAllPurple();
        else
            m_LED.setAllYellow();
    }

    /*
    @Override
    public void execute()
    {
        if ((timer.get() - startTime) >= 0.5)
        {
            SmartDashboard.putBoolean("Cycling", true);
            cycleSpot = false;
            for (int i = 0; i < m_LED.getBuffer().getLength(); i++)
            {
                if (cubeMode)
                {
                    if (cycleSpot)
                    {
                        m_LED.setPurple(i);
                    }
                    else
                    {
                        m_LED.lightOff(i);
                    }
                }
                else
                {
                    if (cycleSpot)
                    {
                        m_LED.setYellow(i);
                    }
                    else
                    {
                        m_LED.lightOff(i);
                    }
                }
                cycleSpot = !cycleSpot;
            }
            m_LED.setData();
            timer.reset();
            startTime = timer.get();
        }
        SmartDashboard.putBoolean("Cycling", false);
    }
    */
}

