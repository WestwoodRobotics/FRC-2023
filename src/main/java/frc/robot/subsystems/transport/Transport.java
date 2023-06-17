package frc.robot.subsystems.transport;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController.AccelStrategy;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  private ProfiledPIDController shoulderController;

  private String currentPos;

  public Transport() {
    shoulderMotorLead.setIdleMode(IdleMode.kBrake);
    shoulderMotorFollow1.setIdleMode(IdleMode.kBrake);
    shoulderMotorFollow2.setIdleMode(IdleMode.kBrake);

    shoulderMotorLead.getPIDController().setP(TransportConstants.shoulderP);

    shoulderMotorLead.setInverted(false);

    elbowMotor.setInverted(true);

    shoulderMotorFollow1.follow(shoulderMotorLead);
    shoulderMotorFollow2.follow(shoulderMotorLead);

    // shoulderMotorLead.enableSoftLimit(SoftLimitDirection.kForward, true);
    // shoulderMotorLead.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // shoulderMotorLead.setSoftLimit(SoftLimitDirection.kForward, (float) TransportConstants.MAX_SHOULDER_ROT);
    // shoulderMotorLead.setSoftLimit(SoftLimitDirection.kReverse, (float) TransportConstants.MIN_SHOULDER_ROT);

    elbowMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    elbowMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    elbowMotor.setSoftLimit(SoftLimitDirection.kForward, (float) TransportConstants.MAX_ELBOW_ROT);
    elbowMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) TransportConstants.MIN_ELBOW_ROT);

    wristMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    wristMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    wristMotor.setSoftLimit(SoftLimitDirection.kForward, (float) TransportConstants.MAX_WRIST_ROT);
    wristMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) TransportConstants.MIN_WRIST_ROT);

    shoulderController = new ProfiledPIDController(TransportConstants.shoulderP, TransportConstants.shoulderI, TransportConstants.shoulderD, new TrapezoidProfile.Constraints(2, 2));
    shoulderMotorLead.getPIDController().setFeedbackDevice(shoulderMotorLead.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle));

    currentPos = "START";

  }

  public void setShoulderMotorPower(double power) {
    double abs_position = getShoulderAbsPosition();
    if(abs_position > 0.9 && abs_position <= 0.9999) {
      power = Math.max(power, 0);
    }
    else if(abs_position > 0.7 && abs_position <= 0.89) {
      power = Math.min(power, 0);
    }
    shoulderMotorLead.set(power);
  }

  public void setElbowMotorPower(double power) {
    elbowMotor.set(power);
  }

  public void setWristMotorPower(double power) {
    wristMotor.set(power);
  }

  public double getShoulderMotorPosition() {
    return shoulderMotorLead.getEncoder().getPosition();
  }

  public double getShoulderAbsPosition() {
    //shoulderMotorLead.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).
    return shoulderMotorLead.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getPosition();
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

  public void setShoulderMotorPosition(float position, float ff) {
    shoulderMotorLead.getPIDController().setReference(position, ControlType.kPosition, 0, ff, ArbFFUnits.kPercentOut);
  }

  public void setShoulderMotorPositionPID(double position)
  {
    shoulderController.calculate(this.getShoulderMotorPosition(), position);
  }

  public String getPos() {
    return currentPos;
  }

  public void setPos(String pos) {
    currentPos = pos;
  }

  public double getShoulderMotorVelocityRPS()
  {
    return shoulderMotorLead.getAbsoluteEncoder(SparkMaxAbsoluteEncoder.Type.kDutyCycle).getVelocity() / 60;
  }

  public double getShoulderMotorVelocity()
  {
    return shoulderMotorLead.getEncoder().getVelocity();
  }

}
