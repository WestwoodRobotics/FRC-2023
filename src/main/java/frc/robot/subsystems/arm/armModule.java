package frc.robot.subsystems.arm;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import frc.robot.Constants.PortConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armModule extends SubsystemBase {
    public WPI_TalonSRX armMotor;
    public int armMotorID;
    public int intakeEncoderID;

    public armModule(){
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
        //TODO: implement
    }

    public void moveDown(double speed){
        //TODO: implement
    }

    public void haltArm(){
        //TODO: implement
    }
}
