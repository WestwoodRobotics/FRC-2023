package frc.robot.subsystems.transport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
import frc.robot.constants.TransportConstants;
import frc.robot.util.Conversions;

public class Transport extends SubsystemBase {
  private final TalonFX shoulderMotorLead = new TalonFX(PortConstants.shoulderLeadMotorPort);
  private final TalonFX shoulderMotorFollow1 = new TalonFX(PortConstants.shoulderFollow1MotorPort);
  private final TalonFX elbowMotor = new TalonFX(PortConstants.elbowMotorPort);
  private final TalonFX wristMotor = new TalonFX(PortConstants.wristMotorPort);


  public Transport() {
    shoulderMotorLead.setNeutralMode(NeutralMode.Brake);
    shoulderMotorFollow1.setNeutralMode(NeutralMode.Brake);

    shoulderMotorLead.setInverted(false);

    elbowMotor.setInverted(true);

    shoulderMotorFollow1.follow(shoulderMotorLead); // Might need to change the false

    shoulderMotorLead.configForwardSoftLimitThreshold(TransportConstants.MAX_SHOULDER_TICKS);
    shoulderMotorLead.configReverseSoftLimitThreshold(TransportConstants.MIN_SHOULDER_TICKS);
    shoulderMotorLead.configReverseSoftLimitEnable(true, 0);
    shoulderMotorLead.configForwardSoftLimitEnable(true, 0);

    elbowMotor.configForwardSoftLimitThreshold(TransportConstants.MAX_ELBOW_TICKS);
    elbowMotor.configReverseSoftLimitThreshold(TransportConstants.MIN_ELBOW_TICKS);
    elbowMotor.configReverseSoftLimitEnable(true, 0);
    elbowMotor.configForwardSoftLimitEnable(true, 0);

    wristMotor.configForwardSoftLimitThreshold(TransportConstants.MAX_WRIST_TICKS);
    wristMotor.configReverseSoftLimitThreshold(TransportConstants.MIN_WRIST_TICKS);
    wristMotor.configReverseSoftLimitEnable(true, 0);
    wristMotor.configForwardSoftLimitEnable(true, 0);
  }

  public void setShoulderMotorPower(double power) {
    shoulderMotorLead.set(ControlMode.PercentOutput, power);
  }

  public double getShoulderMotorPosition() {
    return shoulderMotorLead.getSelectedSensorPosition();
  }

  public void setShoulderMotorPosition(double tick) {
    shoulderMotorLead.set(ControlMode.Position, tick);
  }

  public double getElbowMotorPosition() {
    return elbowMotor.getSelectedSensorPosition();
  }

  public void setElbowMotorPosition(double tick) {
    elbowMotor.set(ControlMode.Position, tick);
  }

  public void setElbowMotorPower(double power) {
    elbowMotor.set(ControlMode.PercentOutput, power);
  }

  public void setWristMotorPower(double power) {
    wristMotor.set(ControlMode.PercentOutput, power);
  }

  public double getShoulderMotorLeadEncoderTicks() {
    return shoulderMotorLead.getSelectedSensorPosition();
  }

  /*public double getShoulderMotorFollow1EncoderTicks() {
    return shoulderMotorFollow1.getSelectedSensorPosition();
  }*/

  public double getElbowMotorEncoderTicks() {
    return elbowMotor.getSelectedSensorPosition();
  }

  public double getWristMotorEncoderTicks() {
    return wristMotor.getSelectedSensorPosition();
  }

  public double getAngleDegrees(TalonFX selectedMotor) {
    if (selectedMotor == shoulderMotorLead) {
        return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kTransportMotorGearRatio);
    }
    else if (selectedMotor == shoulderMotorFollow1) {
        return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kTransportMotorGearRatio);
    }
    else if (selectedMotor == elbowMotor) {
        return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kTransportMotorGearRatio);
    }
    else if (selectedMotor == wristMotor) {
        return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kTransportMotorGearRatio);
    }
    else {
        return -1;
    }

  }

public void printAllMotorRawEncoderTicks(){
    System.out.println("\n Shoulder Motor Lead Encoder Ticks: " + getShoulderMotorLeadEncoderTicks()
                     + "\n Shoulder Motor Follow 1 Encoder Ticks: " + getShoulderMotorFollow1EncoderTicks()
                     + "\n Elbow Motor Encoder Ticks: " + getElbowMotorEncoderTicks()
                     + "\n Wrist Motor Encoder Ticks: " + getWristMotorEncoderTicks());
}


public void printAllMotorCalculatedAngles(){
    System.out.println("\n Shoulder Motor Lead Angle: " + getAngleDegrees(shoulderMotorLead)
                     + "\n Shoulder Motor Follow 1 Angle: " + getAngleDegrees(shoulderMotorFollow1)
                     + "\n Elbow Motor Angle: " + getAngleDegrees(elbowMotor)
                     + "\n Wrist Motor Angle: " + getAngleDegrees(wristMotor));
}

}
