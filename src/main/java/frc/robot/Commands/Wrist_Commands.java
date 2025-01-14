package frc.robot.Commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.Wrist_Constants;
import frc.robot.Subsystems.wrist;


public class Wrist_Commands {
    private wrist wrist;
    private Timer timer;
    public Wrist_Commands(wrist wrist){
        this.wrist = wrist;
        timer = new Timer();
        timer.reset();
    }
    public InstantCommand stop(){
        return new InstantCommand(() -> wrist.stopWristMotor());
    }

    public InstantCommand wristForward(){
        return new InstantCommand(() -> wrist.setWristSpeed(Wrist_Constants.defaultMotorSpeed), wrist);
    }
    public InstantCommand wristReverse(){
        return new InstantCommand(() -> wrist.setWristSpeed(-Wrist_Constants.defaultMotorSpeed), wrist);
    }



}
