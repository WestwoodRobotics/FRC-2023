package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeModule extends SubsystemBase {
  // Motor for the intake mechanism
  private final WPI_VictorSPX intakeMotor;
  // Encoder for measuring the position of the intake motor
  //private double intakeEncoderPosition;

  // Encoder for measuring the velocity of the intake motor
  //private double intakeEncoderVelocity;

  // Constructor for initializing the intake module
  public IntakeModule() {
    intakeMotor = new WPI_VictorSPX(Constants.IntakeConstants.CANID_INTAKE);
    intakeMotor.setInverted(true);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(ControlMode.PercentOutput, power);
  }
}
