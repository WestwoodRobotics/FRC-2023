package frc.robot.constants;

import java.lang.reflect.Array;

public final class TransportConstants {
  public static final float MAX_SHOULDER_ROT = 175.78125f;
  public static final float MAX_ELBOW_ROT = 205.078125f,
                            MAX_WRIST_ROT = 25f;

  public static final float MIN_SHOULDER_ROT = -4.8828125f;
  public static final float MIN_ELBOW_ROT = 0f,
                            MIN_WRIST_ROT = 0f;

  public static final double kTransportMotorGearRatio = 5.0; //TODO: Verify if this is correct

  public static final float WRIST_START_ROT = 0f;
  public static final float WRIST_FLIPPED_ROT = 21.1796875f;

  public static final float HIGH_SHOULDER_ROT = 119.628906f;
  public static final float HIGH_ELBOW_ROT = 97.65625f;
  public static final double HIGH_PERCENT_VOLTS = 0.5;

  public static final float MID_SHOULDER_ROT = 126.953125f;
  public static final float MID_ELBOW_ROT = 97.65625f;
  public static final double MID_PERCENT_VOLTS = 0.5;

  public static final float START_SHOULDER_ROT = 0f;
  public static final float START_ELBOW_ROT = 0f;
  public static final double START_PERCENT_VOLTS = 0.5;

  public static final float GROUND_SHOULDER_ROT = 0.15039062f;  
  public static final float GROUND_ELBOW_ROT = 60.7109375f;
  public static final double GROUND_PERCENT_VOLTS = 0.5;

  public static final float VERTICAL_SHOULDER_ROT = 85.4492188f;
  public static final float VERTICAL_ELBOW_ROT = 25.4140625f;
  public static final double VERTICAL_PERCENT_VOLTS = 0.7;
  
  public static final float SHELF_SHOULDER_ROT = 41.5039062f;
  public static final float SHELF_ELBOW_ROT = 85.4492188f;
  public static final double SHELF_PERCENT_VOLTS = 0.7;

  //public static final int FALCON500_OGTICKS = 2048;
  //public static final int FALCON500_544TICKS = 491520;
}
