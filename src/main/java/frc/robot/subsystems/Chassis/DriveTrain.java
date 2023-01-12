package frc.robot.subsystems.Chassis;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;

public class DriveTrain extends SubsystemBase {
    private  WPI_TalonSRX driveMotor; // Motor for driving
    private  WPI_TalonSRX steerMotor; // Motor for steering
    private  AnalogInput steerEncoder; // Encoder for measuring steer motor position

    /**
     * Constructor for creating a SwerveModule object.
     * @param driveID The ID of the drive motor controller on the CAN bus.
     * @param steerID The ID of the steer motor controller on the CAN bus.
     * @param encoderID The ID of the encoder on the Analog Input module.
     * @return 
     */
    public void SwerveModule(int driveID, int steerID, int encoderID) {
        driveMotor = new WPI_TalonSRX(driveID);
        steerMotor = new WPI_TalonSRX(steerID);
        steerEncoder = new AnalogInput(encoderID);
    }

    /**
     * Set the speed of the drive motor.
     * @param speed Speed of the drive motor in the range [-1, 1].
     */
    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    /**
     * Set the position of the steer motor.
     * @param position The position of the steer motor in the range [-1, 1].
     */
    public void setSteerPosition(double position) {
        // convert position from the range [-1, 1] to angle using the steer range constant
        // double angle = position * SwerveModuleConstants.kSteerRange;
        //steerMotor.set(angle);
    }

    /**
     * Get the position of the steer encoder.
     * @return The position of the steer encoder in the range [0, 1].
     */
    public double getSteerEncoderPosition() {
        return 0;
        //double position = steerEncoder.getVoltage() / SwerveModuleConstants.kEncoderRatio;
        //return position;
    }

    /**
     * Set the default command for this subsystem.
     */
    public void initDefaultCommand() {
        // setDefaultCommand(new MySpecialCommand());
    }
}
