package frc.robot.commands.ArmCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.arm.armModule;

//TODO: EVERYTHING I got it trust
public class UseArm extends CommandBase{
    public armModule m_arm;

    public UseArm(armModule arm){
        m_arm = arm;
        addRequirements(arm);
    }


    @Override
    public void initialize(){

    }

    @Override
    public void execute(){

    }

    @Override
    public void end(boolean interrupted) {
        
    }

    @Override
    public boolean isFinished(){
        return false;
    }


}
