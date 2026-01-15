package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Drivetrain;

public class driveSpinwaysPID extends Command{
private Drivetrain lDrivetrain;
private double m_goalPos = 0;
private double wrappedAngle = 0;
private double m_initialPos = 0;
private double m_currentPos = 0;
private double m_Period = Constants.kDefaultPeriod;
private final ProfiledPIDController m_PIDController;

  /**
   *Creates a new command which will spin the robot in place a given angle.  This function optimizes to minimize the
   * angle travelled to between - pi and pi
   * @param angle in Radians, positive is counter clockwise; 
   * @param drivetrain
*/
  public driveSpinwaysPID(double p_Angle, double p_Period, Drivetrain driveTrain) {
    wrappedAngle = MathUtil.angleModulus(p_Angle); //Wrap the angle to be between -pi and pi
    lDrivetrain = driveTrain;
    m_Period = p_Period;
    addRequirements(lDrivetrain);
    m_PIDController =
    new ProfiledPIDController(
      4, 
      0,
      0, 
      new TrapezoidProfile.Constraints(
                  6,
                    36));
  m_PIDController.setTolerance(.01);
  }

  @Override
  public void initialize() {
    //Run once, at the start of the command
    m_initialPos = lDrivetrain.m_odometry.getPoseMeters().getRotation().getRadians();
    System.out.print("InitPos = ");
    System.out.println(m_initialPos);
    m_goalPos = m_initialPos + wrappedAngle;
  }

  @Override
  public void execute() {
    //run repeatedly, until isFinished() returns true
    m_currentPos = lDrivetrain.m_odometry.getPoseMeters().getRotation().getRadians();
    //m_currDistance = Math.abs(m_currentPos - m_initialPos);
   /*  System.out.print("Current Pos = ");
    System.out.println(m_currentPos);
    System.out.print("Current Dist = ");
    System.out.println(m_currDistance);*/
    lDrivetrain.drive(0,0,
    MathUtil.clamp(m_PIDController.calculate(m_currentPos, m_goalPos), 
      -Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond, 
      Constants.AutoConstants.kMaxAngularSpeedRadiansPerSecond), 
    
    false, 
    m_Period);
  }

  @Override
  public void end(boolean interrupted) {
    //Run once, at the end of the command
    lDrivetrain.drive(0, 0, 0, false, m_Period);

  }

  @Override
  public boolean isFinished() {
    // Determines when to finish the command
    return m_PIDController.atGoal();
   
  }





   
}


