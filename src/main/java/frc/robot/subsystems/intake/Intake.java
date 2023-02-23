package frc.robot.subsystems.intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.CANID_INTAKE);
    public final double initialPosition;

    public Intake(){
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(true);
        intakeMotor.config_kP(0, 0.2);
        intakeMotor.configAllowableClosedloopError(0, 50, 1000);
        intakeMotor.configNeutralDeadband(0.05); //try removing this if the robot acts up.
        
        initialPosition = getPosition();
        //intakeMotor.setSelectedSensorPosition(sensorPos)
    }

    public void setIntakePower(double power){
        intakeMotor.set(ControlMode.PercentOutput, power);
        SmartDashboard.putNumber("intake Motor Position", intakeMotor.getSelectedSensorPosition());
    }


    public void setIntakePosition(double sensorPosition){
        intakeMotor.set(ControlMode.Position, sensorPosition);
    }
    public double getPosition(){
        return intakeMotor.getSelectedSensorPosition();
    }

    public void setIntakeVelocity(double velocity){
        intakeMotor.set(ControlMode.Velocity, velocity);

    }

    public void neutralMode(){
        intakeMotor.setNeutralMode(NeutralMode.Brake);
    }





}
