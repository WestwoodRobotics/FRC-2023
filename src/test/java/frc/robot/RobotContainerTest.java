package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import org.junit.jupiter.api.Test;
// import org.littletonrobotics.junction.inputs.LoggedDriverStation; (AdvantageKit)

public class RobotContainerTest {

  @Test
  public void createRobotContainer() {
    // Set joysticks to silence warnings
    // LoggedDriverStation.getInstance().getJoystickData(0).xbox = true;
    // LoggedDriverStation.getInstance().getJoystickData(1).xbox = true;
    // LoggedDriverStation.getInstance().getJoystickData(2).name = "Generic   USB  Joystick";
    DriverStation.silenceJoystickConnectionWarning(true);

    // Instantiate RobotContainer
    new RobotContainer();
  }
}
