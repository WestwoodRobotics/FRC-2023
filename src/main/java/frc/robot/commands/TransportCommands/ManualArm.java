package frc.robot.commands.TransportCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

public class ManualArm extends CommandBase {

    Transport m_transport;
    XboxController controller;


    public ManualArm(XboxController controller, Transport arm){
        this.controller = controller;

        m_transport = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // This probably shouldn't be in a constant control command
        // Also what do these do?
        // This is fine in constant control command
    
        m_transport.setShoulderMotorPower(
            Conversions.deadZoneSquare(-0.7 * controller.getLeftY(), 0.1));
            

        m_transport.setElbowMotorPower(
           Conversions.deadZoneSquare(0.7 * controller.getRightY(), 0.1));
                           
        m_transport.setWristMotorPower(
            Conversions.deadZoneSquare(-0.5 * controller.getRightX(), 0.1));
        }
    }

