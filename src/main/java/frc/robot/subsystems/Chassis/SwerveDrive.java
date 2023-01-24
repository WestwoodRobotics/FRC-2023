package frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Chassis.SwerveModule;

// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
    // Initialize the 4 swerve modules with the appropriate CAN IDs, inversion states, and encoder offsets
    SwerveModule frontLeftModule = new SwerveModule(0, 1, false, false, 0, 0, false);
    SwerveModule frontRightModule = new SwerveModule(2, 3, false, false, 0, 0, false);
    SwerveModule backLeftModule = new SwerveModule(4, 5, false, false, 0, 0, false);
    SwerveModule backRightModule = new SwerveModule(6, 7, false, false, 0, 0, false);
}
