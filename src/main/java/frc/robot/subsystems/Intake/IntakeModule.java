package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Intake.IntakeModule;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.*;

public class IntakeModule {
    // Motor for the intake mechanism
    private WPI_TalonSRX intakeMotor;
    // Encoder for measuring the position of the intake motor
    private double intakeEncoderPosition;
    // Encoder for measuring the velocity of the intake motor
    private double intakeEncoderVelocity;
    // Constructor for initializing the intake module
    public IntakeModule(int intakeMotorId, int intakeEncoderId){
        intakeMotor = new WPI_TalonSRX(intakeMotorId);
        intakeEncoderPosition = intakeMotor.getSelectedSensorPosition();
        intakeEncoderVelocity = intakeMotor.getSelectedSensorVelocity();
    }
}
