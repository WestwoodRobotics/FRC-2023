package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class IntakeModule extends SubsystemBase {
  // Motor for the intake mechanism
  private final CANSparkMax intakeMotor;

  // Constructor for initializing the intake module
  public IntakeModule() {
    intakeMotor = new CANSparkMax(PortConstants.intakeRollerPort, MotorType.kBrushless);
    intakeMotor.setInverted(true);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }
}
