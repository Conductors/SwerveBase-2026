package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class driveToPositionPID extends Command {
private Drivetrain lDrivetrain;
private Pose2d m_GoalPose = new Pose2d();
private Pose2d m_initialPose = new Pose2d();
private Pose2d m_currentPose = new Pose2d();
private double m_Period = 0;
private final ProfiledPIDController m_PIDControllerX;
private final ProfiledPIDController m_PIDControllerY;
private final ProfiledPIDController m_PIDControllerRot;


  /**
   *Creates a new DriveDistance
   * @param meters
   * @param drivetrain
  */
  public driveToPositionPID(Pose2d p_TargetPose, double p_Period, Drivetrain driveTrain) {
    m_GoalPose = p_TargetPose;
    lDrivetrain = driveTrain;
    m_Period = p_Period;
    addRequirements(lDrivetrain);

    //Gains specific to controlling via 'drive' the robot to a position specified by the user
    m_PIDControllerX =
      new ProfiledPIDController(
        1,
        0,
        0,
        new TrapezoidProfile.Constraints(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,                                       
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));                              
    m_PIDControllerX.setTolerance(.03);  //sets the tolerance for the PID controller, in meters

    m_PIDControllerY =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
                     Constants.AutoConstants.kMaxSpeedMetersPerSecond,                                       
                      Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared));                              
    m_PIDControllerY.setTolerance(.03);  //sets the tolerance for the PID controller, in meters

    m_PIDControllerRot =
      new ProfiledPIDController(
          1,
          0,
          0,
          new TrapezoidProfile.Constraints(
                     Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond,                                       
                      30));                              
    m_PIDControllerRot.setTolerance(.03);  //sets the tolerance for the PID controller, in meters
      
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_initialPose = lDrivetrain.m_odometry.getPoseMeters();

    System.out.println("Initial X= " + m_initialPose.getX());
    System.out.println("Initial Y= " + m_initialPose.getY());
    System.out.println("Initial Rot= " + m_initialPose.getRotation().getRadians());
    System.out.println("Goal X= " + m_GoalPose.getX());
    System.out.println("Goal Y= " + m_GoalPose.getY());
    System.out.println("Goal Rot= " + m_GoalPose.getRotation().getRadians());
    
  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_currentPose = lDrivetrain.m_odometry.getPoseMeters();
    System.out.println("Current X= " + m_currentPose.getX());
    System.out.println("Current Y= " + m_currentPose.getY());
    System.out.println("Current Rot= " + m_currentPose.getRotation().getRadians());
    
    //Drive the robot to the goal position, clamping the error output to +/- max speed
    double xCmd = MathUtil.clamp(m_PIDControllerX.calculate(m_currentPose.getX(), m_GoalPose.getX()), 
                    -Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond);
    double yCmd =   MathUtil.clamp(m_PIDControllerY.calculate(m_currentPose.getY(), m_GoalPose.getY()), 
                    -Constants.AutoConstants.kMaxSpeedMetersPerSecond, 
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond); 
    double thetaCmd = MathUtil.clamp(m_PIDControllerRot.calculate(m_currentPose.getRotation().getRadians(), m_GoalPose.getRotation().getRadians()), 
                    -Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
                    Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecondSquared);
    System.out.print("xCmd");
    System.out.println(xCmd);
    System.out.print("yCmd");
    System.out.println(yCmd);
    System.out.print("thetaCmd");
    System.out.println(thetaCmd);

    lDrivetrain.drive(
      xCmd, yCmd, thetaCmd,
      true, 
      m_Period);
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    lDrivetrain.drive(0, 0, 0, false, m_Period);

  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command - all 3 dimensional PIDs need to be completed
    return (m_PIDControllerX.atGoal() &&
            m_PIDControllerY.atGoal() &&
            m_PIDControllerRot.atGoal());
   
  }
}
