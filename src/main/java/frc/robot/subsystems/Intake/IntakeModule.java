package frc.robot.subsystems.Intake;

import frc.robot.subsystems.Intake.IntakeModule;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

public class IntakeModule {
    // Motor for the intake mechanism
    private WPI_TalonSRX intakeMotor;
    // Encoder for measuring the position of the intake motor
    private double intakeEncoderPosition;
    // Encoder for measuring the velocity of the intake motor
    private double intakeEncoderVelocity;

    // Constructor for initializing the intake module
    public IntakeModule(int intakeMotorId, int intakeEncoderId) {
        intakeMotor = new WPI_TalonSRX(intakeMotorId);
    }

    // Retruns the position of the intake encoder
    public double getIntakeEncoderPosition() {
        return intakeMotor.getSelectedSensorPosition();
        
    }

    // Returns the velocity of the intake encoder
    public double intakeEncoderVelocity() {
        return intakeMotor.getSelectedSensorVelocity();
    }

    // Sets the speed of the intake motor
    public void setIntakeMotor(double speed) {
        intakeMotor.set(speed);
    }
}
