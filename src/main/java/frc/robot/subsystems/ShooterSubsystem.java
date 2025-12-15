// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.seasonspecific.crescendo2024.NoteOnFly;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */
  Distance distance  = Units.Meters.of(0.45);
  double randomValue = 1000.0;
  LinearVelocity linearVelocity;
  Angle distance2 = Units.Radians.of(70);
  IntakeSubsystem intakeSubsystem;
  SwerveDriveSimulation driveTrainSubsystem;



  TalonFX motor1 = new TalonFX(10);

  public ShooterSubsystem(IntakeSubsystem intakeSubsystem, SwerveDriveSimulation driveTrainSubsystem) {
    this.intakeSubsystem = intakeSubsystem;
    this.driveTrainSubsystem = driveTrainSubsystem;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  linearVelocity = Units.MetersPerSecond.of(3000.0/6000.0*20.0);


  }
  public void setShooterSpeed(double speed){
    motor1.set(speed);
  }
  
  public void launchNote(){
    if(!intakeSubsystem.isNoteInsideIntake())  return;

    intakeSubsystem.obtainNoteFromIntake();
    NoteOnFly noteOnFly = new NoteOnFly(        // Specify the position of the chassis when the note is launched
        driveTrainSubsystem.getSimulatedDriveTrainPose().getTranslation(),
        // Specify the translation of the shooter from the robot center (in the shooter’s reference frame)
        new Translation2d(0.2, 0),
        // Specify the field-relative speed of the chassis, adding it to the initial velocity of the projectile
        driveTrainSubsystem.getDriveTrainSimulatedChassisSpeedsRobotRelative(),
        // The shooter facing direction is the same as the robot’s facing direction
        driveTrainSubsystem.getSimulatedDriveTrainPose().getRotation(),
                // Add the shooter’s rotation,
        // Initial height of the flying note
        distance,
        // The launch speed is proportional to the RPM; assumed to be 16 meters/second at 6000 RPM
  linearVelocity,
        // The angle at which the note is launched
        Units.Radian.of(Math.toRadians(70))).enableBecomeNoteOnFieldAfterTouchGround().asSpeakerShotNote(increaseBlueScore);
        SimulatedArena.getInstance().addGamePieceProjectile(noteOnFly);
  }


  Runnable increaseBlueScore = new Runnable() {
    @Override
    public void run() {
      Robot.blueScore++;
    }
};

  public LinearVelocity getVelocity(){
    return Units.MetersPerSecond.of(motor1.getVelocity().getValueAsDouble());
  }
}
