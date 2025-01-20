package frc.robot.subsystems.Gripper;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.Gripper.Intake;


public class Gripper extends SubsystemBase {
    private GripperIO io;

    public Gripper(GripperIO io) {
        this.io = io;
    }

    public boolean isCoralPresent() {
        return io.isCoralPresent();
    }


    public void stop() {
        io.stop();
    }

    public void setIn() {
        io.setIn();
    }

    public void setOut() { 
        io.setOut();
    }

    public Command runIn() {return io.intake();}
    
    public Command runOut() {return io.intake();}
}
