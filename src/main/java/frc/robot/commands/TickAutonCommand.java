package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.swerve.SwerveDrive;

public class TickAutonCommand extends CommandBase {
    private SwerveDrive m_swerveDrive;
    private double targetTicks;
    private double speed;
    private double currentTicks;
    private Timer timer;

    public TickAutonCommand(SwerveDrive swerve, double speed, double targetDriveRotations, 
    double targetSteerDegrees) {
        this.speed = speed;
        this.targetTicks = targetDriveRotations * 2048 * (150 / 7);
        this.currentTicks = 0;
        m_swerveDrive = swerve;
        addRequirements(swerve);
        timer = new Timer();
    }

    @Override
    public void initialize() {
        m_swerveDrive.zeroDrive();
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        //TODO: Not sure if speed should be '-speed' or 'speed' in the .drive method
        m_swerveDrive.drive(0, speed, 0, false);
        currentTicks = m_swerveDrive.getAverageDriveEncoderPositions();
    }

    @Override
    public boolean isFinished() {
        if (timer.get() > 15) {
            DriverStation.reportError("ERROR - Command Took Too Long To Run", false);
            return true;
        }
        return currentTicks >= targetTicks;
    }

    @Override
    public void end(boolean interrupted) {
        m_swerveDrive.drive(0, 0, 0, false);
        m_swerveDrive.zeroDrive();
        m_swerveDrive.zeroTurn();
    }
}