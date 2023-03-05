package frc.robot.subsystems.transport;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.TransportConstants;
import frc.robot.util.Conversions;
import frc.robot.Constants.*;

public class Transport extends SubsystemBase {

    private final TalonFX shoulderMotorLead = new TalonFX(PortConstants.kShoulderLeadMotorPort);
    private final TalonFX shoulderMotorFollow1 = new TalonFX(PortConstants.kShoulderFollow1MotorPort);
    private final TalonFX shoulderMotorFollow2 = new TalonFX(PortConstants.kShoulderFollow2MotorPort);
    private final TalonFX elbowMotor = new TalonFX(PortConstants.kElbowMotorPort);
    private final TalonFX wristMotor = new TalonFX(PortConstants.kWristMotorPort);


    public Transport() {
        shoulderMotorLead.setNeutralMode(NeutralMode.Brake);
        shoulderMotorFollow1.setNeutralMode(NeutralMode.Brake);
        shoulderMotorFollow2.setNeutralMode(NeutralMode.Brake);

        shoulderMotorLead.setInverted(false);
        //shoulderMotorFollow1.setInverted(false);
        //shoulderMotorFollow2.setInverted(false);

        elbowMotor.setInverted(true);

        shoulderMotorFollow1.follow(shoulderMotorLead); // Might need to change the false
        shoulderMotorFollow2.follow(shoulderMotorLead); // Might need to change the false

        shoulderMotorLead.configForwardSoftLimitThreshold(Constants.TransportConstants.MAX_SHOULDER_TICKS);
        shoulderMotorLead.configReverseSoftLimitThreshold(Constants.TransportConstants.MIN_SHOULDER_TICKS);
        shoulderMotorLead.configReverseSoftLimitEnable(false, 0);
        shoulderMotorLead.configForwardSoftLimitEnable(false, 0);

        elbowMotor.configForwardSoftLimitThreshold(Constants.TransportConstants.MAX_ELBOW_TICKS);
        elbowMotor.configReverseSoftLimitThreshold(Constants.TransportConstants.MIN_ELBOW_TICKS);
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


    public double getShoulderMotorLeadEncoderTicks(){
        return shoulderMotorLead.getSelectedSensorPosition();
    }

    public double getShoulderMotorFollow1EncoderTicks(){
        return shoulderMotorFollow1.getSelectedSensorPosition();
    }

    public double getShoulderMotorFollow2EncoderTicks(){
        return shoulderMotorFollow2.getSelectedSensorPosition();
    }

    public double getElbowMotorEncoderTicks(){
        return elbowMotor.getSelectedSensorPosition();
    }

    public double getWristMotorEncoderTicks(){
        return wristMotor.getSelectedSensorPosition();
    }

    public double getAngleDegrees(TalonFX selectedMotor) {
        if (selectedMotor == shoulderMotorLead) {
            return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kShoulderLeadMotorGearRatio);
        } else if (selectedMotor == elbowMotor) {
            return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kElbowMotorGearRatio);
        } else if (selectedMotor == wristMotor) {
            return Conversions.falconToDegrees(selectedMotor.getSelectedSensorPosition(), TransportConstants.kWristMotorGearRatio);
        } else {
            return -1;
        }

      }

    public void printAllMotorRawEncoderTicks(){
        System.out.println("\n Shoulder Motor Lead Encoder Ticks: " + getShoulderMotorLeadEncoderTicks()
                         + "\n Shoulder Motor Follow 1 Encoder Ticks: " + getShoulderMotorFollow1EncoderTicks()
                         + "\n Shoulder Motor Follow 2 Encoder Ticks: " + getShoulderMotorFollow2EncoderTicks()
                         + "\n Elbow Motor Encoder Ticks: " + getElbowMotorEncoderTicks()
                         + "\n Wrist Motor Encoder Ticks: " + getWristMotorEncoderTicks());
    }
    

    public void printAllMotorCalculatedAngles(){
        System.out.println("\n Shoulder Motor Lead Angle (Degrees): " + getAngleDegrees(shoulderMotorLead)
                         + "\n Shoulder Motor Follow 1 Angle (Degrees): " + getAngleDegrees(shoulderMotorFollow1)
                         + "\n Shoulder Motor Follow 2 Angle (Degrees): " + getAngleDegrees(shoulderMotorFollow2)

                         //Value will be negative because we have set the "setInverted" value for Elbow Motor earlier in the file
                         + "\n Elbow Motor Angle (Degrees): " + getAngleDegrees(elbowMotor) 
                         + "\n Wrist Motor Angle (Degrees): " + getAngleDegrees(wristMotor));
    }


}
