package frc.robot.subsystems.transport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstant;

public class Transport extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(TransportConstant.CANID_TRANSPORT);

    public Transport() {
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor.setInverted(false);
    }

    public void setArmMotorPower(double power) {
        armMotor.set(ControlMode.PercentOutput, power);
    }
}
