package frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants.ModuleConstants;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;


public class SwerveDrive extends SubsystemBase {
    private WPI_TalonSRX driveMotor; // Motor for driving
    private WPI_TalonSRX steerMotor; // Motor for steering
    private double driveEncoderPosition; // Encoder for measuring drive motor position
    private double driveEncoderVelocity; // Encoder for measuring drive motor velocity
    private double steerEncoderPosition; // Encoder for measuring drive motor position
    private double steerEncoderVelocity; // Encoder for measuring drive motor velocity
    private PIDController drivePIDController; // PID Controller for the drive motor
    private PIDController steerPIDController; // PID Controller for the steer motor
    private AnalogInput absoluteEncoder; // Absolute Encoder
    private boolean absoluteEncoderReversed;// Boolean for determining if the absolute encoder is reversed
    private double absoluteEncoderOffsetRad;// Offset for the absolute encoder


    /**
     * Constructor for creating a SwerveModule object.
     * @param driveID The ID of the drive motor controller on the CAN bus.
     * @param steerID The ID of the steer motor controller on the CAN bus.
     * @param encoderID The ID of the encoder on the Analog Input module.
     * @return 
     */
    public void SwerveModule(int driveID, int steerID, boolean driveMotorReversed, boolean steerMotorReversed, int absoluteEncoderId, double absoluteEncoderOffset, boolean absoluteEncoderReversed) {
        driveMotor = new WPI_TalonSRX(driveID);
        steerMotor = new WPI_TalonSRX(steerID);
        absoluteEncoder = new AnalogInput(absoluteEncoderId);
    }

    /**
     * Set the speed of the drive motor.
     * @param speed Speed of the drive motor in the range [-1, 1].
     */
    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }
    /**
     * Get the position of the steer encoder.
     * @return The position of the steer encoder in the range [0, 1].
     */
    public double getSteerEncoderPosition() {
        double position = steerMotor.getSelectedSensorPosition();
        return position;
    }

    /**
     * Set the position of the steer motor.
     * @param position The position of the steer motor in the range [-1, 1].
     */
    public void setSteerPosition(double position) {
        // convert position from the range [-1, 1] to angle using the steer range constant
         double angle = position * 0.7078125; // Unsure if the 0.7078125 is the correct steer range constant
        steerMotor.set(angle);
    }

    

    /**
     * Set the default command for this subsystem.
     */
    public void initDefaultCommand() {
        // setDefaultCommand(new MySpecialCommand());
    }
}
