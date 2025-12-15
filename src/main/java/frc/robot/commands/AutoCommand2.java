// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.MLClass;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand2 extends Command {
    SwerveDriveSimulation driveTrainSubsystem;
  VisionSubsystem visionSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  Timer timer;
  boolean done = false;
  boolean done2 = false;

  /** Creates a new AutoCommand2. */
  public AutoCommand2(SwerveDriveSimulation driveSimulation, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveSimulation;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;

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
    if(visionSubsystem.returnArea() > 14 &&  !done){
      done = true;
      intakeSubsystem.setIntakeSpeed(1);
      timer.start();
      
      return;
    }
    if(Robot.DRIVETRAIN_SUBSYSTEM.getMeasuredSpeeds().vxMetersPerSecond == 0 && !done){
         SwerveModuleSimulation[] modules = driveTrainSubsystem.getModules();
      for (SwerveModuleSimulation module : modules) {
        // Get the motor controllers (only do this once during initialization!)
        SimulatedMotorController.GenericMotorController driveController = 
        module.useGenericMotorControllerForDrive();
    SimulatedMotorController.GenericMotorController steerController = 
        module.useGenericControllerForSteer();
  
    driveController.requestVoltage(Volts.of(1.0));
    
    steerController.requestVoltage(Volts.of(0.0));
      }
    }
    if(done){
           SwerveModuleSimulation[] modules = driveTrainSubsystem.getModules();
      for (SwerveModuleSimulation module : modules) {
        // Get the motor controllers (only do this once during initialization!)
        SimulatedMotorController.GenericMotorController driveController = 
        module.useGenericMotorControllerForDrive();
    SimulatedMotorController.GenericMotorController steerController = 
        module.useGenericControllerForSteer();
  
    driveController.requestVoltage(Volts.of(0.0));
    
    steerController.requestVoltage(Volts.of(0.0));
      }
    }
System.out.println("In Auto Command 2");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
    Robot.noDefault = false;
    //new AutoCommand1().schedule();
    Robot.ML_CLASS.calculateNearestNeighbor(driveTrainSubsystem,visionSubsystem,intakeSubsystem, shooterSubsystem, false).schedule();
    SwerveModuleSimulation[] modules = driveTrainSubsystem.getModules();
    for (SwerveModuleSimulation module : modules) {
      // Get the motor controllers (only do this once during initialization!)
      SimulatedMotorController.GenericMotorController driveController = 
      module.useGenericMotorControllerForDrive();
  SimulatedMotorController.GenericMotorController steerController = 
      module.useGenericControllerForSteer();

  driveController.requestVoltage(Volts.of(0.0));
  
  steerController.requestVoltage(Volts.of(0.0));
    }
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    return timer.get() > 2 || Math.abs(visionSubsystem.returnYaw()) > 11 || visionSubsystem.returnArea() < 0.25;
  }




}
  
