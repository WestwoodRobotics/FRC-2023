package frc.robot.commands.TransportCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

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
        ourArm.setArmMotorPower(
            Conversions.deadZoneSquare(controller.getLeftY(), 0.1));
            
        ourArm.setPivot2MotorPower(
            Conversions.deadZoneSquare(controller.getRightY(), 0.1));
    }
}
