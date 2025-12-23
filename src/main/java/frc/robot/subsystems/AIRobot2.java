// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SelfControlledSwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.util.MLClass;

public class AIRobot2 extends SubsystemBase {
  /** Creates a new AIRobot2. */
  public Command currentCommand;
  public Command pendingCommand;
  public final String name;
  public final int id;
  public final MapleSimSwerve driveSimulation;
  public final IntakeSubsystem intakeSubsystem;
  public final ShooterSubsystem shooterSubsystem;
  public final VisionSubsystem visionSubsystem;
  public SendableChooser<String> behaviorChooser = new SendableChooser<>();
  public boolean enabled = true;
  public String lastSelected;

  public AIRobot2(int id,String name, Pose2d pose2d) {
    this.name = name;
    this.id = id;
    driveSimulation = new MapleSimSwerve(false,pose2d);
    intakeSubsystem =   new IntakeSubsystem(driveSimulation.returnSwerveThing());
    shooterSubsystem =   new ShooterSubsystem(intakeSubsystem, driveSimulation.returnSwerveThing());
    this.visionSubsystem =  new VisionSubsystem(driveSimulation.returnSwerveThing(), name);
    behaviorChooser.addOption("Disable", "Disable");
    behaviorChooser.addOption("Enable", "Enable");
    SmartDashboard.putData("AIRobot/Opponent Robot " + id + " Behavior", behaviorChooser);
  }


  public void setCurretCommand(Command newCommand){
    if(newCommand == null) return;
    pendingCommand = newCommand;

  }

 
 
  public void update(){
    String selected = behaviorChooser.getSelected();
    if (selected != null && !selected.equals(lastSelected)) {
        lastSelected = selected;
        if (selected.equals("Enable")) {
            enabled = true;
            Robot.ML_CLASS.calculateNearestNeighbor(
                driveSimulation, visionSubsystem, 
                intakeSubsystem, shooterSubsystem, false, id).schedule();;
        } else {
            enabled = false;
            currentCommand = null;
        }
      }
      /* 
    if(pendingCommand != null){
      currentCommand = pendingCommand;
      currentCommand.initialize();
      pendingCommand = null;
    }
    if(currentCommand == null || !enabled) return;

    currentCommand.execute();
    if(currentCommand.isFinished()){
      currentCommand.end(false);
      currentCommand = null;
    }
    */
  }
    
  
  public void periodic() {
    // This method will be called once per scheduler run

    this.update();
    visionSubsystem.simUpdate();
    Pose3d pose3d = new Pose3d(this.driveSimulation.getPose().getX(),this.driveSimulation.getPose().getY(), 0, new Rotation3d(0,0,this.driveSimulation.getPose().getRotation().getRadians()));
    Logger.recordOutput("Robot/Ai" + name, pose3d);
    if(currentCommand != null){
    System.out.println(currentCommand.getName());
    }
    else System.out.println("Null");
    //this.visionSubsystem.periodic();
    //this.intakeSubsystem.periodic();
    //this.shooterSubsystem.periodic();
  }
}
