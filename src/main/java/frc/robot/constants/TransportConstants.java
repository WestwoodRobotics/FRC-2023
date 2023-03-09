package frc.robot.constants;

import java.lang.reflect.Array;

public final class TransportConstants {
  public static final double MAX_SHOULDER_TICKS = 360000;
  public static final double MAX_ELBOW_TICKS = 420000,
    MAX_WRIST_TICKS = (.5 * 50) * 2048;

  public static final double MIN_SHOULDER_TICKS = 0;
  public static final double MIN_ELBOW_TICKS = 0,
    MIN_WRIST_TICKS = 0;

  public static final double kTransportMotorGearRatio = 5.0; //TODO: Verify if this is correct

  public static final double WRIST_START_TICKS = 0;
  public static final double WRIST_FLIPPED_TICKS = 1000;

  public static final double HIGH_SHOULDER_TICKS = 239520;
  public static final double HIGH_ELBOW_TICKS = 199368;
  public static final double HIGH_PERCENT_VOLTS = 0.5;

  public static final double MID_SHOULDER_TICKS = 260150;
  public static final double MID_ELBOW_TICKS = 199368;
  public static final double MID_PERCENT_VOLTS = 0.5;

  public static final double START_SHOULDER_TICKS = 0;
  public static final double START_ELBOW_TICKS = 0;
  public static final double START_PERCENT_VOLTS = 0.7;

  public static final double GROUND_SHOULDER_TICKS = 8429;  
  public static final double GROUND_ELBOW_TICKS = 101946;
  public static final double GROUND_PERCENT_VOLTS = 0.5;

  public static final double VERTICAL_SHOULDER_TICKS = 173325;
  public static final double VERTICAL_ELBOW_TICKS = 50000;
  public static final double VERTICAL_PERCENT_VOLTS = 0.7;
  
  public static final double SHELF_SHOULDER_TICKS = 75000;
  public static final double SHELF_ELBOW_TICKS = 173000;
  public static final double SHELF_PERCENT_VOLTS = 0.7;

  //public static final int FALCON500_OGTICKS = 2048;
  //public static final int FALCON500_544TICKS = 491520;
}
