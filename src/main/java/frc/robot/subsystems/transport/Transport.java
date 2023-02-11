package frc.robot.subsystems.transport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TransportConstant;

public class Transport extends SubsystemBase {
    private final TalonFX armMotor = new TalonFX(TransportConstant.CANID_TRANSPORT);
    private final TalonFX armMotor1 = new TalonFX(TransportConstant.CANID_TRANSPORT1);
    private final TalonFX armMotor2 = new TalonFX(TransportConstant.CANID_TRANSPORT2);

    public Transport() {
        armMotor.setNeutralMode(NeutralMode.Brake);
        armMotor1.setNeutralMode(NeutralMode.Brake);
        armMotor2.setNeutralMode(NeutralMode.Brake);
        armMotor.setInverted(false);
        armMotor1.setInverted(true);
        armMotor2.setInverted(true);
        armMotor1.follow(armMotor);
        armMotor2.follow(armMotor);
    }

    public void setArmMotorPower(double power) {
        armMotor.set(ControlMode.PercentOutput, power);
    }
    public void setArmMotorPosition(double tick){
        armMotor.set(ControlMode.Position,tick);
    }
    public void addToArmMotorPosition(double tick){
        System.out.println(armMotor.getSelectedSensorPosition() + tick);
        armMotor.set(ControlMode.Position, armMotor.getSelectedSensorPosition() + tick);
        
    }
}
