package frc.robot.constants;

import edu.wpi.first.wpilibj.Filesystem;

public final class FilePathConstants {
  public static final String steerEncoderOffsetPath = Filesystem.getOperatingDirectory().getPath()
    + "/steerEncoderOffsets.txt";
}
