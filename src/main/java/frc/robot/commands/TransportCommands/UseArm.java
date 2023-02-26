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
        if(controller.getAButton()) {
            ourArm.setArmMotorPosition(20000);
        } else if(controller.getBButton()) {
            ourArm.setPivot2MotorPosition(20000);
        } else {
            ourArm.setArmMotorPower(
                Conversions.deadZoneSquare(-1 * controller.getLeftY(), 0.1));
            
            ourArm.setPivot2MotorPower(
                Conversions.deadZoneSquare(controller.getRightY(), 0.1));
                           
            ourArm.setWristMotorPower(
                Conversions.deadZoneSquare(controller.getRightX(), 0.1));
        }
    }
}
