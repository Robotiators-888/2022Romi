package frc.robot.commands;

public package frc.robot.commands;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.Drivetrain;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj.trajectory.constraint.TrajectoryConstraint;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.trajectory.*;

public class GenerateRamsete extends SequentialCommandGroup {
   
private final Drivetrain m_drive = new Drivetrain();
  //private final Trajectory exampleTrajectory;
  
      edu.wpi.first.math.trajectory.Trajectory autoVoltageConstraint =
          new DifferentialDriveVoltageConstraint(
              new SimpleMotorFeedforward(DriveConstants.ksVolts, 
                                         DriveConstants.kvVoltSecondsPerMeter, 
                                         DriveConstants.kaVoltSecondsSquaredPerMeter),
              DriveConstants.kDriveKinematics,
              10);
  
      TrajectoryConfig config =
          new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, 
                               AutoConstants.kMaxAccelerationMetersPerSecondSquared)
              .setKinematics(DriveConstants.kDriveKinematics)
              .addConstraint(autoVoltageConstraint);
  
      // This trajectory can be modified to suit your purposes
      // Note that all coordinates are in meters, and follow NWU conventions.
      // If you would like to specify coordinates in inches (which might be easier
      // to deal with for the Romi), you can use the Units.inchesToMeters() method
      private final  Trajectory exampleTrajectory = TrajectoryGenerator.generateTrajectory(
          // Start at the origin facing the +X direction
          new Pose2d(0, 0, new Rotation2d(0)),
          List.of(
              new Translation2d(0.5, 0.5),
              new Translation2d(1.0, 0.0),
              new Translation2d(0.5,-0.5),
              new Translation2d(0.0, 0.0)
          ),
          new Pose2d(0.0, 0, new Rotation2d(Math.PI)),
          config);
  
      RamseteCommand ramseteCommand = new RamseteCommand(
          exampleTrajectory,
          m_drive::getPose,
          new RamseteController(AutoConstants.kRamseteB, AutoConstants.kRamseteZeta),
          new SimpleMotorFeedforward(DriveConstants.ksVolts, DriveConstants.kvVoltSecondsPerMeter, DriveConstants.kaVoltSecondsSquaredPerMeter),
          DriveConstants.kDriveKinematics,
          m_drive::getWheelSpeeds,
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          new PIDController(DriveConstants.kPDriveVel, 0, 0),
          m_drive::tankDriveVolts,
          m_drive);


  /**
 
   */
  public GenerateRamsete() {
    addCommands(
          // First, we want to reset the drivetrain odometry
        new InstantCommand(() -> m_drive.resetOdometry(exampleTrajectory.getInitialPose()), m_drive),
          // next, we run the actual ramsete command
        ramseteCommand,
          // Finally, we make sure that the robot stops
        new InstantCommand(() -> m_drive.tankDriveVolts(0, 0), m_drive));
  }

}class GenerateRamsete {
    
}
