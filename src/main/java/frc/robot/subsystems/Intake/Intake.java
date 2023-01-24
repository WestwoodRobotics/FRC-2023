package frc.robot.subsystems.Intake;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {

    private final TalonFX intakeMotor = new TalonFX(IntakeConstants.P_INTAKE);

    public Intake(){
        intakeMotor.setNeutralMode(NeutralMode.Brake);
        intakeMotor.setInverted(false);
    }

    public void setIntakeVoltage(double voltage){
        intakeMotor.set(ControlMode.Current, voltage);
    }

    
}
