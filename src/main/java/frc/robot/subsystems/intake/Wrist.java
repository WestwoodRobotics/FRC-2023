package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Wrist extends SubsystemBase {

    private final TalonFX wristMotor = new TalonFX(IntakeConstants.CANID_WRIST_MOTOR);
    private final TalonFX wristMotorTwo = new TalonFX(IntakeConstants.CANID_WRIST_MOTOR_TWO);
    public final double initialPosition;

    public Wrist(){
        wristMotor.setNeutralMode(NeutralMode.Brake);
        wristMotor.setInverted(true);
        wristMotor.config_kP(0, 0.2);
        wristMotor.configAllowableClosedloopError(0, 50, 1000);
        wristMotor.configNeutralDeadband(0.05); //try removing this if the robot acts up.
        
        initialPosition = getPosition();
        //wristMotor.setSelectedSensorPosition(sensorPos)
    }


    public void setWristPosition(double sensorPosition){
        wristMotor.set(ControlMode.Position, sensorPosition);
    }
    public double getPosition(){
        return wristMotor.getSelectedSensorPosition();
    }



    public void neutralMode(){
        wristMotor.setNeutralMode(NeutralMode.Brake);
    }





}
