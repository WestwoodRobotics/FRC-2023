package frc.robot.commands.TransportCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;

public class UseArm extends CommandBase {

    Transport ourArm;
    XboxController controller;

    public UseArm(XboxController controller, Transport arm){
        this.controller = controller;
        ourArm = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        if (controller.getRightTriggerAxis() >= 0.05){
            ourArm.setArmMotorPower(controller.getRightTriggerAxis());
        } else if (controller.getLeftTriggerAxis() >= 0.05){
            ourArm.setArmMotorPower(controller.getLeftTriggerAxis());
        } else {
            ourArm.setArmMotorPower(0);
        }
    }
}
