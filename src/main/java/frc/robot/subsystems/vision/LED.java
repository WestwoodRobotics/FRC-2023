package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase
{
  private AddressableLED m_LED;
  private AddressableLEDBuffer m_buffer;
  private boolean cycleSpot = true;
  

  public LED(AddressableLED led, AddressableLEDBuffer buff)
  {
    m_LED = led;
    m_buffer = buff;
    m_LED.start();
  }

  private void setColor(int R, int G, int B)
  {
    if ((R <= 255) && (G <= 255) && (B <= 255))
    {
      for (int i = 0; i < m_buffer.getLength(); i++)
      {
        m_buffer.setRGB(i, R, G, B);
      }
    }
    m_LED.setData(m_buffer);
  }

  public void setGreen()
  {
    this.setColor(0, 255, 0);
  }

  public void setOrange()
  {
    this.setColor(255, 103, 0);
  }

  public void setPurple()
  {
    this.setColor(255, 0, 255);
  }

  public void setYellow()
  {
    this.setColor(255, 255, 0);
  }

  public void lightsOff()
  {
    this.setColor(0, 0, 0);
  }

  public void cycle (boolean tele, boolean cubeMode)
  {
    for (int i = 0; i < m_buffer.getLength(); i++)
    {
      if (tele)
      {
        if (cubeMode)
        {
          if (cycleSpot)
          {
            this.setPurple();
          }
          else
          {
            this.lightsOff();
          }
        }
        else
        {
          if (cycleSpot)
          {
            this.setYellow();
          }
          else
          {
            this.lightsOff();
          }
        }
      }
      else
      {
        if (cycleSpot)
          {
            this.setOrange();
          }
          else
          {
            this.setGreen();
          }
      }
      cycleSpot = !cycleSpot;
    }
  }

}
