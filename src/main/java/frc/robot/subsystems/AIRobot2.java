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
  public final String name;
  public final int id;
  public final MapleSimSwerve driveSimulation= new MapleSimSwerve(false);
  public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem(driveSimulation.returnSwerveThing());
  public final ShooterSubsystem shooterSubsystem = new ShooterSubsystem(intakeSubsystem, driveSimulation.returnSwerveThing());
  public final VisionSubsystem visionSubsystem;
  public SendableChooser<Command> behaviorChooser = new SendableChooser<Command>();
  public AIRobot2(int id,String name) {
    this.name = name;
    this.id = id;
    this.visionSubsystem =  new VisionSubsystem(driveSimulation.returnSwerveThing(), name);
    behaviorChooser.addOption("Disable",new Command(){
      @Override
      public void initialize() {
       setCurretCommand(null);
      }
    });
    behaviorChooser.addOption("Enable", new Command() {
      @Override
      public void initialize() {
        setCurretCommand(Robot.ML_CLASS.calculateNearestNeighbor(driveSimulation.returnSwerveThing(), visionSubsystem, intakeSubsystem, shooterSubsystem, false,id));
      }
    });
    behaviorChooser.onChange(Command::initialize);
    SmartDashboard.putData("AIRobot/Opponent Robot " + id + " Behavior", behaviorChooser);
    //SimulatedArena.getInstance().addDriveTrainSimulation(driveSimulation.returnSwerveThing());
  }


  public void setCurretCommand(Command newCommand){
    if(newCommand == null) return;
    currentCommand = newCommand;
    currentCommand.initialize();
  }

 

  public void update(){
    if(currentCommand == null) return;

    currentCommand.execute();
    if(currentCommand.isFinished()){
      currentCommand.end(false);
    }
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    this.update();
    visionSubsystem.simUpdate();
    Pose3d pose3d = new Pose3d(this.driveSimulation.getPose().getX(),this.driveSimulation.getPose().getY(), 0, new Rotation3d(0,0,this.driveSimulation.getPose().getRotation().getRadians()));
    Logger.recordOutput("Robot/Ai" + name, pose3d);
  }
}
