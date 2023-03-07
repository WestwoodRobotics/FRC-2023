package frc.robot.constants;

public final class TransportConstants {
  public static final double MAX_SHOULDER_TICKS = 360000;
  public static final double MAX_ELBOW_TICKS = 420000,
    MAX_WRIST_TICKS = (.5 / 50) * 2048;

  public static final double MIN_SHOULDER_TICKS = 0;
  public static final double MIN_ELBOW_TICKS = 0,
    MIN_WRIST_TICKS = 0;

  public static final double kTransportMotorGearRatio = 5.0; //TODO: Verify if this is correct

  public static final double HIGH_SHOULDER_TICKS = 247000;
  public static final double HIGH_ELBOW_TICKS = 160000;
  public static final double MID_SHOULDER_TICKS = 272000;
  public static final double MID_ELBOW_TICKS = 186000;
  public static final double START_SHOULDER_TICKS = 0;
  public static final double START_ELBOW_TICKS = 0;
  public static final double GROUND_SHOULDER_TICKS = 0; //wrong
  public static final double GROUND_ELBOW_TICKS = 125000; //wrong
  public static final double VERTICAL_SHOULDER_TICKS = 170000;
  public static final double VERTICAL_ELBOW_TICKS = 0;
  public static final double SHELF_SHOULDER_TICKS = 66000;
  public static final double SHELF_ELBOW_TICKS = 193000;
  //public static final int FALCON500_OGTICKS = 2048;
  //public static final int FALCON500_544TICKS = 491520;
}
