package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.util.Conversions;

public class UseIntake extends CommandBase {

    IntakeModule intake;
    XboxController controller;

    public UseIntake(XboxController controller, IntakeModule i){
        this.controller = controller;
        intake = i;
        addRequirements(i);
    }

    @Override
    public void execute() {

        intake.setIntakePower(
            Conversions.deadZoneSquare(controller.getRightTriggerAxis()-controller.getLeftTriggerAxis(), 0.1));
            
    }
}
