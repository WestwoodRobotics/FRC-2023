package frc.robot.commands;

import java.time.Instant;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WrapperCommand;
import frc.robot.commands.transport.IntakeModes;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.subsystems.swerve.SwerveDrive;
import frc.robot.subsystems.transport.Transport;
import frc.robot.subsystems.vision.LED;

public class UpdateLEDs extends CommandBase
{
    private LED led;
    private IntakeModule intake;

    public UpdateLEDs(LED led, IntakeModule intake)
    {
        this.led = led;
        this.intake = intake;
        addRequirements(led, intake);
    }

    @Override
    public void initialize()
    {
      if (intake.getIntakeMode() == 0) {
        led.setYellow();
      }
      else if (intake.getIntakeMode() == 1) {
        led.setOrange();
      }
    }

    @Override
    public boolean isFinished()
    {
      return true;
        //return true;
    }
}
