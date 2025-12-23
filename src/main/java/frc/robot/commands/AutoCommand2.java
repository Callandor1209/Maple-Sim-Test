// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;


import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.MapleSimSwerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.MLClass;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand2 extends Command {
    MapleSimSwerve driveTrainSubsystem;
  VisionSubsystem visionSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  Timer timer;
  boolean done = false;
  boolean done2 = false;
  int id;

  /** Creates a new AutoCommand2. */
  public AutoCommand2(MapleSimSwerve driveSimulation, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveSimulation;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.id = id;
    addRequirements(driveSimulation,visionSubsystem,intakeSubsystem,shooterSubsystem);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = new Timer();
    timer.reset();
    Robot.noDefault = true;
    done = false;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double area = visionSubsystem.returnArea();
    double yaw = visionSubsystem.returnYaw();
    ChassisSpeeds measured = driveTrainSubsystem.getMeasuredSpeeds();
    
    System.out.println("Area: " + area + " | Yaw: " + yaw + " | Done: " + done);
    System.out.println("Measured VX: " + measured.vxMetersPerSecond + " | VY: " + measured.vyMetersPerSecond);
    System.out.println(visionSubsystem.returnArea());
    if(visionSubsystem.returnArea() > 13 &&  !done){
      done = true;
      intakeSubsystem.setIntakeSpeed(1);
      timer.start();

      
      return;
    }
    if(!done){

      ChassisSpeeds speeds = new ChassisSpeeds(1,0,0);
      driveTrainSubsystem.drive(speeds, false, false);
    }
    if(done){
      ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);
      driveTrainSubsystem.drive(speeds, false, false);
    }

System.out.println("In Auto Command 2");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intakeSubsystem.setIntakeSpeed(0);
    shooterSubsystem.setShooterSpeed(1);
    shooterSubsystem.launchNote();
    shooterSubsystem.setShooterSpeed(0);
    if(!interrupted){
      if(Math.abs(visionSubsystem.returnYaw()) > 11 || visionSubsystem.returnArea() < 0.25){
        new AutoCommand1(driveTrainSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem, id).schedule();
        return;
      }
    Robot.noDefault = false;
    Command nextCommand = Robot.ML_CLASS.calculateNearestNeighbor(driveTrainSubsystem,visionSubsystem,intakeSubsystem, shooterSubsystem, false,id);
    nextCommand.schedule();
      ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);
      driveTrainSubsystem.drive(speeds, false, false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.get() > 2 || Math.abs(visionSubsystem.returnYaw()) > 11 || visionSubsystem.returnArea() < 0.25;
  }




}
  
