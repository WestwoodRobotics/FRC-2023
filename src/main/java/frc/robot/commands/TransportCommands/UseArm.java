package frc.robot.commands.TransportCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

public class UseArm extends CommandBase {

    Transport m_transport;
    XboxController controller;

    public UseArm(XboxController controller, Transport arm){
        this.controller = controller;
        m_transport = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // This probably shouldn't be in a constant control command
        // Also what do these do?
        if(controller.getAButton()) {
            m_transport.setShoulderMotorPosition(20000);
        } 
        else if(controller.getBButton()) {
            m_transport.setElbowMotorPosition(20000);
        } 
        // This is fine in constant control command
        else {
            m_transport.setShoulderMotorPower(
                Conversions.deadZoneSquare(-1 * controller.getLeftY(), 0.1));
            
            m_transport.setElbowMotorPower(
                Conversions.deadZoneSquare(controller.getRightY(), 0.1));
                           
            m_transport.setWristMotorPower(
                Conversions.deadZoneSquare(controller.getRightX(), 0.1));
        }
    }
}
