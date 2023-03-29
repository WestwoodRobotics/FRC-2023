package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
import frc.robot.constants.TransportConstants;

public class IntakeModule extends SubsystemBase {
  // Motor for the intake mechanism
  private final CANSparkMax intakeMotor;
  private int intakeMode = 0;
  //private int inverted = 1;

  // Constructor for initializing the intake module
  public IntakeModule() {
    intakeMotor = new CANSparkMax(PortConstants.intakeRollerPort, MotorType.kBrushless);
    intakeMotor.setInverted(false);
  }

  public void setIntakePower(double power) {
    intakeMotor.set(power);
  }

  public int intakeInverted(int mode) {
    if(mode == 2){
      return -1;
    } else {
      return 1;
    }
  }

  public int getIntakeMode() {
    return intakeMode % 3;
  }

  public void incrementMode() {
    intakeMode++;
  }

  public float getRotValue(){
    if(getIntakeMode() == 0) {
      return TransportConstants.WRIST_START_ROT;
    }
    else if(getIntakeMode() == 1) {
      return TransportConstants.WRIST_HALF_ROT;
    }
    else
    {
      return TransportConstants.WRIST_FLIPPED_ROT;
    }
  }
}
