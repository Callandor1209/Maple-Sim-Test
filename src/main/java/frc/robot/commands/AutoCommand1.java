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
import frc.robot.subsystems.MapleSimSwerve;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class AutoCommand1 extends Command {
  /** Creates a new autoCommand. */
  MapleSimSwerve driveTrainSubsystem;
  VisionSubsystem visionSubsystem;
  IntakeSubsystem intakeSubsystem;
  ShooterSubsystem shooterSubsystem;
  boolean robotRed;
  int id;

  private Timer timer;
  double x = 0;
  double y = 0;
  Command followPath;

  public AutoCommand1(MapleSimSwerve driveSimulation, VisionSubsystem visionSubsystem, IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem,int id) {
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
      if(DriverStation.getAlliance().get() == Alliance.Red){
    robotRed = true;
  }
  else{
    robotRed = false;
  }
    Robot.noDefault = true;
    ChassisSpeeds speeds = new ChassisSpeeds(1,0,0);
    driveTrainSubsystem.drive(speeds, false, false);;;
 

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
    
    if(visionSubsystem.returnYaw() > 10 && visionSubsystem.returnYaw() < 50 + 50  && visionSubsystem.returnArea() >= 0.25){
      ChassisSpeeds speeds = new ChassisSpeeds(0.7,0,0);

      driveTrainSubsystem.drive(speeds, false, false);;;
      if(followPath != null){
      followPath.cancel();
      }
    }
    else{
      ChassisSpeeds speeds = new ChassisSpeeds(0,0,1);
      driveTrainSubsystem.drive(speeds, false, false);;;
 
    }
    if(driveTrainSubsystem.getMeasuredSpeeds().omegaRadiansPerSecond ==0 && driveTrainSubsystem.getMeasuredSpeeds().vxMetersPerSecond ==0 && driveTrainSubsystem.getMeasuredSpeeds().vyMetersPerSecond == 0){
      ChassisSpeeds speeds = new ChassisSpeeds(0,-1,0);
      driveTrainSubsystem.drive(speeds, false, false);
    }
    System.out.println("In Auto Command 1");
    System.out.println("Yaw: " + visionSubsystem.returnYaw());




  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Ending");
    if(!interrupted){
      ChassisSpeeds speeds = new ChassisSpeeds(0,0,0);
      driveTrainSubsystem.drive(speeds, false, false);
    Command nextCommand = new AutoCommand2(driveTrainSubsystem, visionSubsystem, intakeSubsystem, shooterSubsystem,id);
    nextCommand.schedule();
    System.out.println("Ended");
    }

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(visionSubsystem.returnYaw()) < 10 && visionSubsystem.returnYaw() != 0;
  }



  public void generateNewPath(){
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
      new Pose2d(driveTrainSubsystem.getPose().getX(),driveTrainSubsystem.getPose().getY(), new Rotation2d()),
      new Pose2d(x,y,new Rotation2d())
    );
    PathConstraints pathConstraints = new PathConstraints(1, 1, 180, 360); //this is just pulled from pathplanner defaults for now
    PathPlannerPath path = new PathPlannerPath(waypoints, pathConstraints, null, new GoalEndState(0,new Rotation2d()));
     try {
      followPath =  new FollowPathCommand(
        path,
        () -> driveTrainSubsystem.getPose(),
        () -> driveTrainSubsystem.getMeasuredSpeeds(),
        (speeds,feedforwards) -> driveTrainSubsystem.drive(speeds, false, false),
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
  }
}
