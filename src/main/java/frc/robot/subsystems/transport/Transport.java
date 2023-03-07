package frc.robot.subsystems.transport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransportConstants;

public class Transport extends SubsystemBase {
  private final TalonFX shoulderMotorLead = new TalonFX(TransportConstants.CANID_SHOULDER_LEAD);
  //private final TalonFX shoulderMotorFollow1 = new TalonFX(TransportConstants.CANID_SHOULDER_FOLLOW_1);
  private final TalonFX shoulderMotorFollow2 = new TalonFX(TransportConstants.CANID_SHOULDER_FOLLOW_2);
  private final TalonFX elbowMotor = new TalonFX(TransportConstants.CANID_ELBOW);
  private final TalonFX wristMotor = new TalonFX(TransportConstants.CANID_WRIST);


  public Transport() {
    shoulderMotorLead.setNeutralMode(NeutralMode.Brake);
    //shoulderMotorFollow1.setNeutralMode(NeutralMode.Brake);
    shoulderMotorFollow2.setNeutralMode(NeutralMode.Brake);

    shoulderMotorLead.setInverted(false);
    //shoulderMotorFollow1.setInverted(false);
    //shoulderMotorFollow2.setInverted(false);

    elbowMotor.setInverted(true);

    //shoulderMotorFollow1.follow(shoulderMotorLead); // Might need to change the false
    shoulderMotorFollow2.follow(shoulderMotorLead); // Might need to change the false

    shoulderMotorLead.configForwardSoftLimitThreshold(Constants.TransportConstants.MAX_SHOULDER_TICKS);
    shoulderMotorLead.configReverseSoftLimitThreshold(Constants.TransportConstants.MIN_SHOULDER_TICKS);
    shoulderMotorLead.configReverseSoftLimitEnable(true, 0);
    shoulderMotorLead.configForwardSoftLimitEnable(true, 0);

    elbowMotor.configForwardSoftLimitThreshold(Constants.TransportConstants.MAX_ELBOW_TICKS);
    elbowMotor.configReverseSoftLimitThreshold(Constants.TransportConstants.MIN_ELBOW_TICKS);
    elbowMotor.configReverseSoftLimitEnable(true, 0);
    elbowMotor.configForwardSoftLimitEnable(true, 0);

    wristMotor.configForwardSoftLimitThreshold(Constants.TransportConstants.MAX_WRIST_TICKS);
    wristMotor.configReverseSoftLimitThreshold(Constants.TransportConstants.MIN_WRIST_TICKS);
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

  public double getShoulderMotorFollow2EncoderTicks() {
    return shoulderMotorFollow2.getSelectedSensorPosition();
  }

  public double getElbowMotorEncoderTicks() {
    return elbowMotor.getSelectedSensorPosition();
  }

  public double getWristMotorEncoderTicks() {
    return wristMotor.getSelectedSensorPosition();
  }


}
