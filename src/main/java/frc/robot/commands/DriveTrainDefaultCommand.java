// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Second;


import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Robot;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.util.AngularPositionHolder;
import frc.robot.util.Constants;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveTrainDefaultCommand extends Command {
  /** Creates a new Drive. */
      private SwerveDrive drive;
      double maxVelocity = Units.MetersPerSecond.of(4.45).in(MetersPerSecond);
      double maxRotationalVelocity = Rotations.per(Second).of(0.75).in(RadiansPerSecond);


    public DriveTrainDefaultCommand(
            SwerveDrive drive
    ) {
        this.drive = drive;
        addRequirements(drive);
    }

    

    @Override
    public void execute() {
   Rotation2d robotRotation = Robot.DRIVETRAIN_SUBSYSTEM.getPose().getRotation();
    double targetRotation = robotRotation.getRadians();
    double controllerDirection = DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red ? 1 : -1;
    double x = Robot.m_driverController.getRawAxis(1) * controllerDirection;
    double y = Robot.m_driverController.getRawAxis(0) * controllerDirection;
    // playstation controller
    double r = findR(Robot.m_driverController.getRawAxis(5), Robot.m_driverController.getRawAxis(2));
    // xbox controller
     //double r = findR(Robot.m_driverController.getRawAxis(5) ,Robot.m_driverController.getRawAxis(4));
    x = squareAndKeepSign(x);
    y = squareAndKeepSign(y);
    r = squareAndKeepSign(r);
    x = applyDeadband(x);
    y = applyDeadband(y);
    r = applyRotationalDeadband(r);
    x = x * maxVelocity;
    y = y * maxVelocity;
    r = r * maxRotationalVelocity;

    if (Robot.m_driverController.L1().getAsBoolean()) {
      x = x * Constants.CUT_POWER;
      y = y * Constants.CUT_POWER;
      r = r * Constants.CUT_POWER;

    }

    ChassisSpeeds speeds = new ChassisSpeeds(-x,-y,-r);
    if(!Robot.noDefault){
    drive.drive(speeds, true,true);
    }
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drive.drive(new ChassisSpeeds(0,0,0), true, true);
  }



    private double applyDeadband(double x) {
    if (Math.abs(x) > 0.0144) {
      double percentage = (Math.abs(x) - 0.0144) / (1 - 0.0144);
      double sign = Math.copySign(1, x);
      double power = 0.07 / maxVelocity * sign
          + (1 - 0.07 / maxVelocity) * percentage * sign;
      return power;
    } else {
      return 0;
    }
  }

  private double applyRotationalDeadband(double x) {
    if (Math.abs(x) > 0.0144) {
      double percentage = (Math.abs(x) - 0.0144) / (1 - 0.0144);
      double sign = Math.copySign(1, x);
      double power = 0.01 / maxRotationalVelocity * sign
          + (1 - 0.01 / maxRotationalVelocity) * percentage * sign;
      return power;
    } else {
      return 0;
    }
  }
  private double squareAndKeepSign(double num) {
    double sign = Math.copySign(1, num);
    return num * num * sign;
  }

  public double findR(double axis5, double axis2) {
    if (axis2 > 0) {
      if (axis2 + Math.abs(axis5) > 1) {
        return 1;
      }
      return axis2 + Math.abs(axis5);
    }
    if (axis2 - Math.abs(axis5) < -1) {
      return -1;
    }
    return axis2 + -(Math.abs(axis5));
  }



  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
