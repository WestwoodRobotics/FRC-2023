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

        if (controller.getAButton()){
            while (m_transport.getWristMotorEncoderTicks() < 2048){
                m_transport.setWristMotorPower(0.5);
            }
            m_transport.setShoulderMotorPower(0);
        }
        else if (controller.getBButton()){
            while (m_transport.getWristMotorEncoderTicks() != 0){
                m_transport.setWristMotorPower(-0.5);
            }
            m_transport.setWristMotorPower(0);
        }

        if (controller.getXButton()){
            while (m_transport.getShoulderMotorLeadEncoderTicks() < 1024){ //TODO: Update to actual scoring ticks
                m_transport.setShoulderMotorPower(0.5);
            }
            m_transport.setElbowMotorPower(0);
        }
        else if (controller.getYButton()){
            while (m_transport.getElbowMotorEncoderTicks() != 0){
                m_transport.setElbowMotorPower(-0.5);
            }
            m_transport.setElbowMotorPower(0);
        }

        // if(controller.getAButton()) {
        //     m_transport.setShoulderMotorPosition(20000);
        // } 
        // else if(controller.getBButton()) {
        //     m_transport.setElbowMotorPosition(20000);
        // } 
        // // This is fine in constant control command
        // else {
        //     m_transport.setShoulderMotorPower(
        //         Conversions.deadZoneSquare(-1 * controller.getLeftY(), 0.1));
            
        //     m_transport.setElbowMotorPower(
        //         Conversions.deadZoneSquare(controller.getRightY(), 0.1));
                           
        //     m_transport.setWristMotorPower(
        //         Conversions.deadZoneSquare(controller.getRightX(), 0.1));
        // }
    }
}
