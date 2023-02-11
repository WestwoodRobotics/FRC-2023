package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.PortConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Arm extends SubsystemBase {
    public WPI_TalonSRX armMotor;
    public int armMotorID;
    public int intakeEncoderID;

    public Arm(){
        this.armMotorID = PortConstants.kArmMotorPort;
        this.intakeEncoderID = PortConstants.kArmEncoderPort;
        armMotor = new WPI_TalonSRX(armMotorID);
        
    }

    public void setSpeed(double speed){
        armMotor.set(speed);
    }

    public double getEncoderPosition(){
        return armMotor.getSelectedSensorPosition();
    }

    //TODO: lot
    private void setPosition(double position){
        //TODO: implement
    }

    public void moveUp(double speed){
        //if arm position is less than 500 set speed to speed
        
    }

    public void moveDown(double speed){
        armMotor.set(-speed);
    }

    public void haltArm(){
        armMotor.setNeutralMode(NeutralMode.Brake);
    }

    //this does nothing
    private double calculateArmTorque(){

        return -9000;
    }
}
