package frc.robot.subsystems.swerve;

// import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.PortConstants;
//import odometry from wpi



// This class represents the swerve drive system, which is composed of 4 swerve modules (one for each wheel)
public class SwerveDrive extends SubsystemBase {
    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;



    private Odometry odometry;
    protected Gyro gyro;

    public SwerveDrive() {
        setName("SwerveDrive");
       gyro = new Gyro();
        // initialize swerve mods (possibly move into a list for conciseness eventually)
        frontLeftModule = new SwerveModule(PortConstants.kFrontLeftDriveMotorPort,
                PortConstants.kFrontLeftSteerMotorPort,
                false, false, PortConstants.kFrontLeftCANCoderPort, 0, 0);
        frontRightModule = new SwerveModule(PortConstants.kFrontRightDriveMotorPort,
                PortConstants.kFrontRightSteerMotorPort, false, false, PortConstants.kFrontRightCANCoderPort, 0, 1);
        backLeftModule = new SwerveModule(PortConstants.kBackLeftDriveMotorPort, PortConstants.kBackLeftSteerMotorPort,
                false, false, PortConstants.kBackLeftCANCoderPort, 0, 2);
        backRightModule = new SwerveModule(PortConstants.kBackRightDriveMotorPort,
                PortConstants.kBackRightSteerMotorPort,
                false, false, PortConstants.kBackRightCANCoderPort, 0, 3);

        // initialize classes which require Swerve
       odometry = new Odometry(this);
    }

    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {

        SwerveModuleState[] swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
                fieldRelative
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, gyro.getYaw())
                        : new ChassisSpeeds(xSpeed, ySpeed, rot));

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, AutoConstants.kMaxSpeedMetersPerSecond);

        frontLeftModule.setDesiredState(swerveModuleStates[1]);
        frontRightModule.setDesiredState(swerveModuleStates[3]);
        backLeftModule.setDesiredState(swerveModuleStates[0]);
        backRightModule.setDesiredState(swerveModuleStates[2]);
    }

    public void zeroDrive() {
        frontLeftModule.zeroDriveMotor();
        frontRightModule.zeroDriveMotor();
        backLeftModule.zeroDriveMotor();
        backRightModule.zeroDriveMotor();
    }

    public Rotation2d getHeading() {
        return odometry.getHeading();
    }

    public Pose2d getPoseMeters() {
        return odometry.getPoseMeters();
    }

    public SwerveModulePosition[] getPositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        positions[1] = frontLeftModule.getPosition();
        positions[3] = frontRightModule.getPosition();
        positions[0] = backLeftModule.getPosition();
        positions[2] = backRightModule.getPosition();
        return positions;
    }

    public void setStates(SwerveModuleState[] states) {
        frontLeftModule.setDesiredState(states[1]);
        frontRightModule.setDesiredState(states[3]);
        backLeftModule.setDesiredState(states[0]);
        backRightModule.setDesiredState(states[2]);
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        states[1] = frontLeftModule.getState();
        states[3] = frontRightModule.getState();
        states[0] = backLeftModule.getState();
        states[2] = backRightModule.getState();
        return states;
    }

    public void saveEncoderOffsets() {
        frontLeftModule.setEncoderOffset();
        frontRightModule.setEncoderOffset();
        backLeftModule.setEncoderOffset();
        backRightModule.setEncoderOffset();
    }

    public void resetAllEncoders() {
        System.out.println("Encoders reset!");
        frontLeftModule.resetEncoders();
        frontRightModule.resetEncoders();
        backLeftModule.resetEncoders();
        backRightModule.resetEncoders();
        saveEncoderOffsets();
    }

    public double getFrontLeftModuleSteerMotorTicks() {
        return frontLeftModule.getSteerMotorEncoderTicks();
    }

    public double getFrontRightModuleSteerMotorTicks() {
        return frontRightModule.getSteerMotorEncoderTicks();
    }

    public double getBackLeftModuleSteerMotorTicks() {
        return backLeftModule.getSteerMotorEncoderTicks();
    }

    public double getBackRightModuleSteerMotorTicks() {
        return backRightModule.getSteerMotorEncoderTicks();
    }

    public String printDriveTrainSteerMotorDegrees() {
        double frontLeftSteerMotorEncoderTicksTODegrees = getFrontLeftModuleSteerMotorTicks() * 360 / 4096;
        double frontRightSteerMotorEncoderTicksTODegrees = getFrontRightModuleSteerMotorTicks() * 360 / 4096;
        double backLeftSteerMotorEncoderTicksTODegrees = getBackLeftModuleSteerMotorTicks() * 360 / 4096;
        double backRightSteerMotorEncoderTicksTODegrees = getBackRightModuleSteerMotorTicks() * 360 / 4096;
        return("Front Left Steer Motor: " + frontLeftSteerMotorEncoderTicksTODegrees + " degrees" + "\n" +
                "Front Right Steer Motor: " + frontRightSteerMotorEncoderTicksTODegrees + " degrees" + "\n" +
                "Back Left Steer Motor: " + backLeftSteerMotorEncoderTicksTODegrees + " degrees" + "\n" +
                "Back Right Steer Motor: " + backRightSteerMotorEncoderTicksTODegrees + " degrees");
    }

    public void resetPose(Pose2d pose) {
        odometry.resetOdometry(pose);
    }

    @Override
    public void periodic() {
        odometry.update();
    }
}
