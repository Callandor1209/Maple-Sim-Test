// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.standard.RequestingUserName;

import org.ironmaple.simulation.IntakeSimulation;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.AbstractDriveTrainSimulation;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class IntakeSubsystem extends SubsystemBase {
  
  /** Creates a new IntakeSubsystem. */
      private final IntakeSimulation intakeSimulation;
  public final TalonFX intakeMotor = new TalonFX(0);
  public IntakeSubsystem(AbstractDriveTrainSimulation driveTrain) {
    intakeSimulation = IntakeSimulation.OverTheBumperIntake("Note", driveTrain, Units.Meter.of(0.2),Units.Meter.of(0.2), IntakeSimulation.IntakeSide.BACK, 1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if(intakeSimulation !=null){
      if(intakeMotor.get() > 0.01){
        intakeSimulation.startIntake();
        System.out.println("Intake Starting");
      }
      else
      intakeSimulation.stopIntake();
    }
  }



public boolean isNoteInsideIntake() {
    return intakeSimulation.getGamePiecesAmount() != 0; 
}




  public void setIntakeSpeed(double speed){
    intakeMotor.set(speed);

  }

  public double getIntakeSpeed(){
    return intakeMotor.get();
  }
  

  public void obtainNoteFromIntake(){
    intakeSimulation.obtainGamePieceFromIntake();
  }
}
