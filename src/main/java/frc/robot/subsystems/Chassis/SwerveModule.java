package frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.RobotDriveBase.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;


public class SwerveModule {
    private final WPI_TalonSRX driveMotor; // Motor for driving
    private final WPI_TalonSRX steerMotor; // Motor for steering
    private final AnalogInput driveEncoder; // Encoder for measuring steer motor position
    private final AnalogInput steerEncoder; // Encoder for measuring drive motor position
    private final AnalogInput steerPIDController;// PID Controller for the steer motor
    private final AnalogInput absoluteEncoder;// Absolute Encoder for the steer motor
    private final boolean absoluteEncoderReversed;// Boolean for determining if the absolute encoder is reversed
    private final double absoluteEncoderOffsetRad;// Offset for the absolute encoder
    public SwerveModule(int driveMotorId, int steerMotorId, boolean driveMotorReversed, boolean steerMotorReversed,
    int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed ){
        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new AnalogInput(absoluteEncoderId);

        driveMotor = new WPI_TalonSRX(driveMotorId, MotorType.kBrushless);
        steerMotor = new WPI_TalonSRX(steerMotorId, MotorType.kBrushless);

        driveMotor.setInverted(driveMotorReversed);
        steerMotor.setInverted(steerMotorReversed);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        //driveEncoder.setPositionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter);
        //driveEncoder.setVelocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);
        //steerEncoder.setPositionConversionFactor(ModuleConstants.ksteerEncoderRot2Rad);
        //steerEncoder.setVelocityConversionFactor(ModuleConstants.ksteerEncoderRPM2RadPerSec);

        //steerPidController = new PIDController(ModuleConstants.kPsteer, 0, 0);
        //steerPidController.enableContinuousInput(-Math.PI, Math.PI);

        //resetEncoders();
    }
    
}
