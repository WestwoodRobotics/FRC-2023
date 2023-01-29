package frc.robot.subsystems.Chassis;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.*;

// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
    // Initialize the 4 swerve modules with the appropriate CAN IDs, inversion states, and encoder offsets
    SwerveModule frontLeftModule = new SwerveModule(PortConstants.kFrontLeftDriveMotorPort, PortConstants.kFrontLeftSteerMotorPort, false, false, PortConstants.kFrontLeftCANCoderPort, 0, false, 1);
    SwerveModule frontRightModule = new SwerveModule(PortConstants.kFrontRightDriveMotorPort, PortConstants.kFrontRightSteerMotorPort, false, false, PortConstants.kFrontRightCANCoderPort, 0, false, 2);
    SwerveModule backLeftModule = new SwerveModule(PortConstants.kBackLeftDriveMotorPort, PortConstants.kBackLeftSteerMotorPort, false, false, PortConstants.kBackLeftCANCoderPort, 0, false, 3);
    SwerveModule backRightModule = new SwerveModule(PortConstants.kBackRightDriveMotorPort, PortConstants.kBackRightSteerMotorPort, false, false, PortConstants.kBackRightCANCoderPort, 0, false, 4);
    
}
