package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.PortConstants;
import frc.robot.constants.TransportConstants;

public class Transport extends SubsystemBase {
  private final CANSparkMax shoulderMotorLead = new CANSparkMax(PortConstants.shoulderLeadMotorPort,
      MotorType.kBrushless);
  private final CANSparkMax shoulderMotorFollow1 = new CANSparkMax(PortConstants.shoulderFollow1MotorPort,
      MotorType.kBrushless);
  private final CANSparkMax shoulderMotorFollow2 = new CANSparkMax(PortConstants.shoulderFollow2MotorPort,
      MotorType.kBrushless);
  private final CANSparkMax elbowMotor = new CANSparkMax(PortConstants.elbowMotorPort, MotorType.kBrushless);
  private final CANSparkMax wristMotor = new CANSparkMax(PortConstants.wristMotorPort, MotorType.kBrushless);

  public Transport() {
    shoulderMotorLead.setIdleMode(IdleMode.kBrake);
    shoulderMotorFollow1.setIdleMode(IdleMode.kBrake);
    shoulderMotorFollow2.setIdleMode(IdleMode.kBrake);

    shoulderMotorLead.getPIDController().setP(TransportConstants.shoulderP);

    shoulderMotorLead.setInverted(false);

    elbowMotor.setInverted(true);

    shoulderMotorFollow1.follow(shoulderMotorLead);
    shoulderMotorFollow2.follow(shoulderMotorLead);

    shoulderMotorLead.enableSoftLimit(SoftLimitDirection.kForward, true);
    shoulderMotorLead.enableSoftLimit(SoftLimitDirection.kReverse, true);
    shoulderMotorLead.setSoftLimit(SoftLimitDirection.kForward, (float) TransportConstants.MAX_SHOULDER_ROT);
    shoulderMotorLead.setSoftLimit(SoftLimitDirection.kReverse, (float) TransportConstants.MIN_SHOULDER_ROT);

    elbowMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elbowMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elbowMotor.setSoftLimit(SoftLimitDirection.kForward, (float) TransportConstants.MAX_ELBOW_ROT);
    elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) TransportConstants.MIN_ELBOW_ROT);

    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) TransportConstants.MAX_WRIST_ROT);
    wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) TransportConstants.MIN_WRIST_ROT);

  }

  public void setShoulderMotorPower(double power) {
    shoulderMotorLead.set(power);
  }

  public void setElbowMotorPower(double power) {
    elbowMotor.set(power);
  }

  public void setWristMotorPower(double power) {
    wristMotor.set(
        power);
  }

  public double getShoulderMotorPosition() {
    return shoulderMotorLead.getEncoder().getPosition();
  }

  public double getElbowMotorPosition() {
    return elbowMotor.getEncoder().getPosition();
  }

  public double getWristMotorPosition() {
    return wristMotor.getEncoder().getPosition();
  }

  public boolean zeroTransportEncoders() {
    elbowMotor.getEncoder().setPosition(0);
    shoulderMotorLead.getEncoder().setPosition(0);
    wristMotor.getEncoder().setPosition(0);
    return true;
  }

  public void printAllMotorRawEncoderTicks() {
    System.out.println("\n Shoulder Motor Lead Encoder Ticks: " + getShoulderMotorPosition()
        + "\n Shoulder Motor Follow 1 Encoder Ticks: " + "\n Elbow Motor Encoder Ticks: " + getElbowMotorPosition()
        + "\n Wrist Motor Encoder Ticks: " + getWristMotorPosition());
  }

  public void setShoulderMotorPosition(float position) {
    shoulderMotorLead.getPIDController().setReference(position, ControlType.kPosition);
  }
}
