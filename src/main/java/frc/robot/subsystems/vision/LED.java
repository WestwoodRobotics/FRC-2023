package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LED extends SubsystemBase
{
  private AddressableLED m_LED;
  private AddressableLEDBuffer m_buffer;
  

  public LED(AddressableLED led, AddressableLEDBuffer buff)
  {
    m_LED = led;
    m_buffer = buff;
    m_LED.start();
  }

  public void setGreen(int index)
  {
    m_buffer.setRGB(index, 0, 255, 0);
  }

  public void setOrange(int index)
  {
    m_buffer.setRGB(index, 255, 103, 0);
  }

  public void setPurple(int index)
  {
    m_buffer.setRGB(index, 255, 0, 255);
  }

  public void setAllPurple()
  {
    for (int i = 0; i < m_buffer.getLength(); i++)
    {
      setPurple(i);
    }
    m_LED.setData(m_buffer);
  }

  public void setAllYellow()
  {
    for (int i = 0; i < m_buffer.getLength(); i++)
    {
      setYellow(i);
    }
    m_LED.setData(m_buffer);
  }

  public void setYellow(int index)
  {
    m_buffer.setRGB(index, 255, 255, 0);
  }

  public void lightOff(int index)
  {
    m_buffer.setRGB(index, 0, 0, 0);
  }

  public AddressableLEDBuffer getBuffer()
  {
    return m_buffer;
  }

  public void setData()
  {
    m_LED.setData(m_buffer);
  }

}
