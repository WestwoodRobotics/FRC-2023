package frc.robot.subsystems.intake;


import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class IntakeModuleCIM extends SubsystemBase {
  // Motor for the intake mechanism
  private final CANSparkMax intakeMotor;

  // Constructor for initializing the intake module
  public IntakeModuleCIM () {
    intakeMotor = new CANSparkMax(PortConstants.intakeRollerPort, MotorType.kBrushed);
    intakeMotor.setInverted(true);
    
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }
}
