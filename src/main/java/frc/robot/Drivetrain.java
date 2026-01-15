// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.path.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Represents a swerve drive style drivetrain. */
public class Drivetrain extends SubsystemBase {


  private final Translation2d m_frontLeftLocation = new Translation2d(0.303, 0.303);
  private final Translation2d m_frontRightLocation = new Translation2d(0.303, -0.303);
  private final Translation2d m_backLeftLocation = new Translation2d(-0.303, 0.303);
  private final Translation2d m_backRightLocation = new Translation2d(-0.303, -0.303);

  private final SwerveModule m_frontLeft = new SwerveModule(Constants.kFL_DriveChannel, Constants.kFL_TurnChannel,  
                                                    Constants.kFL_TurnEncoderChannel, Constants.kFL_TurnEncoderOffset);      
  private final SwerveModule m_frontRight = new SwerveModule(Constants.kFR_DriveChannel, Constants.kFR_TurnChannel, 
                                                    Constants.kFR_TurnEncoderChannel, Constants.kFR_TurnEncoderOffset);      
  private final SwerveModule m_backLeft = new SwerveModule(Constants.kBL_DriveChannel, Constants.kBL_TurnChannel,  
                                                    Constants.kBL_TurnEncoderChannel, Constants.kBL_TurnEncoderOffset);      
  private final SwerveModule m_backRight = new SwerveModule(Constants.kBR_DriveChannel, Constants.kBR_TurnChannel, 
                                                    Constants.kBR_TurnEncoderChannel, Constants.kBR_TurnEncoderOffset);      

  private RobotConfig mRobotconfig;

  public final ADXRS450_Gyro m_gyro = new ADXRS450_Gyro();

  private double m_ServeRot = 0;

  public final SwerveDriveKinematics m_kinematics =
      new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

  public final SwerveDriveOdometry m_odometry =
      new SwerveDriveOdometry(
          m_kinematics,
          new Rotation2d(m_gyro.getRotation2d().getRadians()),
          new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_backLeft.getPosition(),
            m_backRight.getPosition()
          });

  public Pose2d robotPose2d = new Pose2d();

  public Drivetrain() {
    m_gyro.reset();

    configurePathPlanner();

    //This EventTrigger is supposed to run when the PathPlanner path crosses a mid point
    new EventTrigger("testEvent1").whileTrue(new InstantCommand(() -> System.out.println("PathPlanner Event Trigger")));


    //Create a NamedCommand so that we can use this in the event tag
    NamedCommands.registerCommand("testEvent1", new InstantCommand(() -> System.out.println("PathPlanner Named Command")));

    SmartDashboard.putData("Swerve Drive", new Sendable() {
      @Override
      public void initSendable(SendableBuilder builder) {
        builder.setSmartDashboardType("SwerveDrive");

        builder.addDoubleProperty("Front Left Angle", () -> m_frontLeft.getAdjustedAngle(), null);
        builder.addDoubleProperty("Front Left Velocity", () -> m_frontLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Front Right Angle", () -> m_frontRight.getAdjustedAngle(), null);
        builder.addDoubleProperty("Front Right Velocity", () -> m_frontRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Left Angle", () -> m_backLeft.getAdjustedAngle(), null);
        builder.addDoubleProperty("Back Left Velocity", () -> m_backLeft.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Back Right Angle", () -> m_backRight.getAdjustedAngle(), null);
        builder.addDoubleProperty("Back Right Velocity", () -> m_backRight.getState().speedMetersPerSecond, null);

        builder.addDoubleProperty("Robot Angle", () -> m_ServeRot, null);
      }
    });
    
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed Speed of the robot in the x direction (forward).
   * @param ySpeed Speed of the robot in the y direction (sideways).
   * @param rot Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the field.
   */
  public void drive(
      double xSpeed, double ySpeed, double rot, boolean fieldRelative, double periodSeconds) {
      var swerveModuleStates =
        m_kinematics.toSwerveModuleStates(
            ChassisSpeeds.discretize(
                fieldRelative
                    ? ChassisSpeeds.fromFieldRelativeSpeeds(
                        xSpeed, ySpeed, rot, new Rotation2d(m_gyro.getRotation2d().getRadians()))
                    : new ChassisSpeeds(xSpeed, ySpeed, rot),
                periodSeconds));
                   
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.kMaxRobotSpeed);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);

    m_ServeRot = rot;          

    double[] moduleDesiredStates = {
      swerveModuleStates[0].angle.getDegrees(),
      swerveModuleStates[0].speedMetersPerSecond,
      swerveModuleStates[1].angle.getDegrees(),
      swerveModuleStates[1].speedMetersPerSecond,
      swerveModuleStates[2].angle.getDegrees(),
      swerveModuleStates[2].speedMetersPerSecond,
      swerveModuleStates[3].angle.getDegrees(),
      swerveModuleStates[3].speedMetersPerSecond
    };

    publishToDashboard();
    
    SmartDashboard.putNumberArray("DesiredStates", moduleDesiredStates);
    SmartDashboard.putNumber("FL_Desired_Angle", swerveModuleStates[0].angle.getRadians());
    SmartDashboard.putNumber("FR_Desired_Angle", swerveModuleStates[1].angle.getRadians());
    SmartDashboard.putNumber("BL_Desired_Angle", swerveModuleStates[2].angle.getRadians());
    SmartDashboard.putNumber("BR_Desired_Angle", swerveModuleStates[3].angle.getRadians());



  }

  /** Updates the field relative position of the robot. */
  public void updateOdometry() {
    m_odometry.update(
        new Rotation2d(m_gyro.getRotation2d().getRadians()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        });
  }

  public Pose2d getPose() {
    return robotPose2d;
  }

  /**
   * Sets the swerve ModuleStates.
   *
   * @param desiredStates The desired SwerveModule states.
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.kMaxRobotSpeed);
        m_frontLeft.setDesiredState(desiredStates[0]);
        m_frontRight.setDesiredState(desiredStates[1]);
        m_backLeft.setDesiredState(desiredStates[2]);
        m_backRight.setDesiredState(desiredStates[3]);
  }

    /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  public void resetOdometry(Pose2d pose) {
    m_odometry.resetPosition(
        new Rotation2d(m_gyro.getRotation2d().getRadians()),
        new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_backLeft.getPosition(),
          m_backRight.getPosition()
        },
        pose);
  }

  public void publishToDashboard() {

    
    double[] moduleOptDesStates = {
      m_frontLeft.getOptState().angle.getRadians(),
      m_frontLeft.getOptState().speedMetersPerSecond,
      m_frontRight.getOptState().angle.getRadians(),
      m_frontRight.getOptState().speedMetersPerSecond,
      m_backLeft.getOptState().angle.getRadians(),
      m_backLeft.getOptState().speedMetersPerSecond,
      m_backRight.getOptState().angle.getRadians(),
      m_backRight.getOptState().speedMetersPerSecond
    };

    double[] moduleActualStates = {
      m_frontLeft.getState().angle.getDegrees(),
      m_frontLeft.getState().speedMetersPerSecond,
      m_frontRight.getState().angle.getDegrees(),
      m_frontRight.getState().speedMetersPerSecond,
      m_backLeft.getState().angle.getDegrees(),
      m_backLeft.getState().speedMetersPerSecond,
      m_backRight.getState().angle.getDegrees(),
      m_backRight.getState().speedMetersPerSecond
    };

    SmartDashboard.putNumberArray("ActualStates", moduleActualStates);
    SmartDashboard.putNumberArray("OptDesiredStates", moduleOptDesStates);
    SmartDashboard.putNumber("FL adj Pos", m_frontLeft.getAdjustedAngle());
    SmartDashboard.putNumber("FR adj Pos", m_frontRight.getAdjustedAngle());
    SmartDashboard.putNumber("BL adj Pos", m_backLeft.getAdjustedAngle());
    SmartDashboard.putNumber("BR adj Pos", m_backRight.getAdjustedAngle());
    SmartDashboard.putNumber("FL abs Pos", m_frontLeft.getAbsAngle());
    SmartDashboard.putNumber("FR abs Pos", m_frontRight.getAbsAngle());
    SmartDashboard.putNumber("BL abs Pos", m_backLeft.getAbsAngle());
    SmartDashboard.putNumber("BR abs Pos", m_backRight.getAbsAngle());


    SmartDashboard.putNumberArray("FL Actual V", m_frontLeft.getActualMotorVoltageData());
    SmartDashboard.putNumberArray("FR Actual V", m_frontRight.getActualMotorVoltageData());
    SmartDashboard.putNumberArray("BL Actual V", m_backLeft.getActualMotorVoltageData());
    SmartDashboard.putNumberArray("BR Actual V", m_backLeft.getActualMotorVoltageData());


    SmartDashboard.putNumberArray("FL Command", m_frontLeft.getMotorVoltageData());
    SmartDashboard.putNumberArray("FR Command", m_frontRight.getMotorVoltageData());
    SmartDashboard.putNumberArray("BL Command", m_backLeft.getMotorVoltageData());
    SmartDashboard.putNumberArray("BR Command", m_backRight.getMotorVoltageData());

    SmartDashboard.putNumber("FL Position", m_frontLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("FR Position", m_frontRight.getPosition().distanceMeters);
    SmartDashboard.putNumber("BL Position", m_backLeft.getPosition().distanceMeters);
    SmartDashboard.putNumber("BR Position", m_backRight.getPosition().distanceMeters);

    
  }

  public ChassisSpeeds getRobotChassisSpeeds() {
    return m_kinematics.toChassisSpeeds(        
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_backLeft.getState(),
          m_backRight.getState()
      );
  }

  public double[] getActualModuleStates() {
    return new double[] {
      m_frontLeft.getState().angle.getDegrees(),
      m_frontLeft.getState().speedMetersPerSecond,
      m_frontRight.getState().angle.getDegrees(),
      m_frontRight.getState().speedMetersPerSecond,
      m_backLeft.getState().angle.getDegrees(),
      m_backLeft.getState().speedMetersPerSecond,
      m_backRight.getState().angle.getDegrees(),
      m_backRight.getState().speedMetersPerSecond };
    
  }

  

  /**
   *  DriveRobotRelative uses chassis speeds inputs to ddrive the robot relative to robot's orientation
   * at a default period rate.
   * @param speeds ChassisSpeeds object containing the vx, vy, and omega values
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
    drive(speeds.vxMetersPerSecond,
          speeds.vyMetersPerSecond,
          speeds.omegaRadiansPerSecond,
          false,
          Constants.kDefaultPeriod);
  }

  public void configurePathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably store this in your Constants file
    
    try{
      mRobotconfig = RobotConfig.fromGUISettings();
    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }
 
    AutoBuilder.configure(
            this::getPose, 
            this::resetOdometry,
            this::getRobotChassisSpeeds,
            this::driveRobotRelative,
            new PPHolonomicDriveController( // Holonomic drive controller                                                 
                        new PIDConstants(Constants.AutoConstants.kPP_PXController, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(Constants.AutoConstants.kPP_PThetaController, 0.0, 0.0)), // Rotation PID constants
            mRobotconfig, 
            () -> {
              return false; // Boolean supplier that controls when the path will be mirrored for the red alliance
            },
            this);
        

  }
  public Command getPathPlannerCommand() {
    try{
        // Load the path you want to follow using its name in the GUI
        //PathPlannerPath path = PathPlannerPath.fromPathFile("testPath1"); //forward 1.5m, then stop
        PathPlannerPath path = PathPlannerPath.fromPathFile("fwd_left");    //forward 1.5m, the left 1.5m (heading stays 0 deg)
        //PathPlannerPath path = PathPlannerPath.fromPathFile("spin_in_place");


        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
    } catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
    }
  }
}
