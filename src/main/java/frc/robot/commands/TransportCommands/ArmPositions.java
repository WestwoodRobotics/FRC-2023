package frc.robot.commands.TransportCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.transport.Transport;
import frc.robot.util.Conversions;

public class ArmPositions extends CommandBase {

    Transport m_transport;
    double shoulderPos;
    double elbowPos;


    public ArmPositions(double shoulderPosition, double elbowPosition, Transport arm){
        this.shoulderPos = shoulderPosition;
        this.elbowPos = elbowPosition;

        m_transport = arm;
        addRequirements(arm);
    }

    @Override
    public void execute() {
        // This probably shouldn't be in a constant control command
        // Also what do these do?
        m_transport.setShoulderMotorPosition(shoulderPos);
        m_transport.setElbowMotorPosition(elbowPos);
    }

    @Override
    public boolean isFinished(){
        return (Math.abs(m_transport.getShoulderMotorPosition() - shoulderPos) < 20) && (Math.abs(m_transport.getElbowMotorPosition() - elbowPos) < 20);
    }
}

