package frc.robot.constants;

public final class PortConstants {

  // Absolute encoder ports
  public static final int frontLeftEncoderPort = 1;
  public static final int frontRightEncoderPort = 3;
  public static final int backLeftEncoderPort = 4;
  public static final int backRightEncoderPort = 2;

  // PWM ports for the drive motors
  public static final int frontLeftDriveMotorPort = 17;
  public static final int backLeftDriveMotorPort = 13;
  public static final int frontRightDriveMotorPort = 16;
  public static final int backRightDriveMotorPort = 12;

  // PWM ports for the steer motors
  public static final int frontLeftSteerMotorPort = 19;
  public static final int backLeftSteerMotorPort = 14;
  public static final int frontRightSteerMotorPort = 18;
  public static final int backRightSteerMotorPort = 11;

  // USB port of the driver controller
  public static final int primaryControllerPort = 0;
  public static final int secondaryControllerPort = 1;

  // PWM Port for Pigeon (Gyroscope)
  public static final int pigeonPort = 5;

  // PWM ports for Transport subsystem
  public static final int shoulderLeadMotorPort = 20;
  public static final int shoulderFollow1MotorPort = 22;
  public static final int elbowMotorPort = 29;
  public static final int wristMotorPort = 34;
  public static final int intakeRollerPort = 23;
}
