package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;

public class IntakeModule extends SubsystemBase {
  // Motor for the intake mechanism
  private final WPI_TalonFX intakeMotor;

  // Constructor for initializing the intake module
  public IntakeModule() {
    intakeMotor = new WPI_TalonFX(PortConstants.intakeRollerPort);
    intakeMotor.setInverted(true);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }
}
