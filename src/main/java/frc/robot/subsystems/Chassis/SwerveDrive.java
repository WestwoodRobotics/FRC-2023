package frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Chassis.SwerveModule;
import frc.robot.Constants.*;

// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
    // Initialize the 4 swerve modules with the appropriate CAN IDs, inversion states, and encoder offsets
    SwerveModule frontLeftModule = new SwerveModule(ModuleConstants.kFrontLeftDriveMotorPort, ModuleConstants.kFrontLeftSteerMotorPort, false, false, 0, 0, false);
    SwerveModule frontRightModule = new SwerveModule(ModuleConstants.kFrontRightDriveMotorPort, ModuleConstants.kFrontRightSteerMotorPort, false, false, 0, 0, false);
    SwerveModule backLeftModule = new SwerveModule(ModuleConstants.kBackLeftDriveMotorPort, ModuleConstants.kBackLeftSteerMotorPort, false, false, 0, 0, false);
    SwerveModule backRightModule = new SwerveModule(ModuleConstants.kBackRightDriveMotorPort, ModuleConstants.kBackRightSteerMotorPort, false, false, 0, 0, false);
}
