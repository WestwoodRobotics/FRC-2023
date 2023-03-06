package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.intake.IntakeModule;
import frc.robot.util.Conversions;

public class UseIntake extends CommandBase {

    IntakeModule intake;
    XboxController controller1, controller2;

    public UseIntake(XboxController controller1, XboxController controller2, IntakeModule i){
        this.controller1 = controller1;
        this.controller2 = controller2;
        intake = i;
        addRequirements(i);
    }

    @Override
    public void execute() {

        if ((controller1.getRightTriggerAxis() != 0) || (controller1.getLeftTriggerAxis() !=0)){
            intake.setIntakePower(
                Conversions.deadZoneSquare(controller1.getRightTriggerAxis()-controller1.getLeftTriggerAxis(), 0.1));
        } else {
            intake.setIntakePower(
                Conversions.deadZoneSquare(controller2.getRightTriggerAxis()-controller2.getLeftTriggerAxis(), 0.1));
        }
        //intake.setIntakePower(
        //    Conversions.deadZoneSquare(controller1.getRightTriggerAxis()-controller1.getLeftTriggerAxis(), 0.1));
            
    }
}
