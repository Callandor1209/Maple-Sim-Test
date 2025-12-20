// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Volts;

import java.io.IOException;
import java.util.List;

import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.SwerveModuleSimulation;
import org.ironmaple.simulation.motorsims.SimulatedMotorController;
import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Robot;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.ArrayClass;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand1 extends Command {
  /** Creates a new autoCommand. */
  SwerveDriveSimulation driveTrainSubsystem;
  VisionSubsystem visionSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  boolean robotRed;
  int id;

  private Timer timer;
  double x = 0;
  double y = 0;
  Command followPath;

  public AutoCommand1(SwerveDriveSimulation driveSimulation, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,int id) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveTrainSubsystem = driveSimulation;
    this.visionSubsystem = visionSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.shooterSubsystem = shooterSubsystem;
    this.id = id;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
      if(DriverStation.getAlliance().get() == Alliance.Red){
    robotRed = true;
  }
  else{
    robotRed = false;
  }
    Robot.noDefault = true;
    ChassisSpeeds speeds = new ChassisSpeeds(1,0,0);
    driveTrainSubsystem.setRobotSpeeds(speeds);
 

timer = new Timer();
timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(followPath != null){
      followPath.execute();
      if(followPath.isFinished()){
        followPath.end(false);
        followPath = null;
      }
      return;
    }
    x = Math.random() * 20;
    y = Math.random() * 10;
    if(y > 8 ){
     y = 2;
    }
    if(x >  16){
     x = 2;
    }
    if(timer.get() > 6){

      List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses( 
      new Pose2d(driveTrainSubsystem.getSimulatedDriveTrainPose().getX(),driveTrainSubsystem.getSimulatedDriveTrainPose().getY(), new Rotation2d()),
      new Pose2d(x,y,new Rotation2d())
    );
    PathConstraints pathConstraints = new PathConstraints(1, 1, 180, 360); //this is just pulled from pathplanner defaults for now
    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0,new Rotation2d()));
     try {
      followPath =  new FollowPathCommand(
        path,
        () -> driveTrainSubsystem.getSimulatedDriveTrainPose(),
        () -> driveTrainSubsystem.getDriveTrainSimulatedChassisSpeedsFieldRelative(),
        (speeds,feedforwards) -> driveTrainSubsystem.setRobotSpeeds(speeds),
        Robot.holoConfig, 
        RobotConfig.fromGUISettings(),
        () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red
 );
    } catch (IOException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    } catch (ParseException e) {
      // TODO Auto-generated catch block
      e.printStackTrace();
    }

     followPath.initialize();

     timer.restart();
    }
    if(visionSubsystem.returnYaw() > 10 && visionSubsystem.returnYaw() < 50 + 50  && isBallAllianceColor() && visionSubsystem.returnArea() >= 0.25){
      ChassisSpeeds speeds = new ChassisSpeeds(0.4,0,0);
    
 ChassisSpeeds speeds2 = new ChassisSpeeds();
 driveTrainSubsystem.setRobotSpeeds(speeds2);
      if(followPath != null){
      followPath.cancel();
      }
    }
    else{
      ChassisSpeeds speeds = new ChassisSpeeds(0,0,1);
      driveTrainSubsystem.setRobotSpeeds(speeds);
 
    }
    System.out.println("In Auto Command 1");



  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!interrupted){
 
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
    Command nextCommand = new AutoCommand2(driveTrainSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem,id);
    ArrayClass.aiRobot2Array[id].setCurretCommand(nextCommand);
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(visionSubsystem.returnYaw()) < 10 && visionSubsystem.returnYaw() != 0 && isBallAllianceColor();
  }

  private boolean isBallAllianceColor(){
    if(Robot.isSimulation()){
      if(robotRed){
        return visionSubsystem.returnTagId() >= 100;
      }
      else{

        return visionSubsystem.returnTagId() < 100;
      }
    }
    return false;
  }
}
