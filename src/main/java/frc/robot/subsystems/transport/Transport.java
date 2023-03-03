package frc.robot.subsystems.transport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransportConstant;

public class Transport extends SubsystemBase {
    private final TalonFX shoulderMotorLead = new TalonFX(TransportConstant.CANID_SHOULDER_LEAD);
    private final TalonFX shoulderMotorFollow1 = new TalonFX(TransportConstant.CANID_SHOULDER_FOLLOW_1);
    private final TalonFX shoulderMotorFollow2 = new TalonFX(TransportConstant.CANID_SHOULDER_FOLLOW_2);
    private final TalonFX elbowMotor = new TalonFX(TransportConstant.CANID_ELBOW);
    private final TalonFX wristMotor = new TalonFX(TransportConstant.CANID_WRIST);


    public Transport() {
        shoulderMotorLead.setNeutralMode(NeutralMode.Brake);
        shoulderMotorFollow1.setNeutralMode(NeutralMode.Brake);
        shoulderMotorFollow2.setNeutralMode(NeutralMode.Brake);

        shoulderMotorLead.setInverted(false);
        //shoulderMotorFollow1.setInverted(false);
        //shoulderMotorFollow2.setInverted(false);

        elbowMotor.setInverted(true);

        shoulderMotorFollow1.follow(shoulderMotorLead, false); // Might need to change the false
        shoulderMotorFollow2.follow(shoulderMotorLead, false); // Might need to change the false

        shoulderMotorLead.configForwardSoftLimitThreshold(Constants.TransportConstant.MAX_SHOULDER_TICKS);
        shoulderMotorLead.configReverseSoftLimitThreshold(Constants.TransportConstant.MIN_SHOULDER_TICKS);
        shoulderMotorLead.configReverseSoftLimitEnable(false, 0);
        shoulderMotorLead.configForwardSoftLimitEnable(false, 0);

        elbowMotor.configForwardSoftLimitThreshold(Constants.TransportConstant.MAX_ELBOW_TICKS);
        elbowMotor.configReverseSoftLimitThreshold(Constants.TransportConstant.MIN_ELBOW_TICKS);
        elbowMotor.configReverseSoftLimitEnable(true, 0);
        elbowMotor.configForwardSoftLimitEnable(true, 0);
    }

    public void setShoulderMotorPower(double power) {
        shoulderMotorLead.set(ControlMode.PercentOutput, power);
    }


    public void setShoulderMotorPosition(double tick){
        shoulderMotorLead.set(ControlMode.Position, tick);
    }

    public void setElbowMotorPower(double power) {
        elbowMotor.set(ControlMode.PercentOutput, power);
    }

    public void setWristMotorPower(double power) {
        wristMotor.set(ControlMode.PercentOutput, power);
    }

    public void setElbowMotorPosition(double tick) {
        elbowMotor.set(ControlMode.Position, tick);
    }
}
