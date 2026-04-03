// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.AnalogInput;
//import frc.robot.subsystems.limelightTracking;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {

    private final AnalogInput fan = new AnalogInput(3);

    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

    public Robot() {

        //tell limelight to track for april tags
        LimelightHelpers.setPipelineIndex("limelight-turret", 0);
        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
        m_timeAndJoystickReplay.update();
        CommandScheduler.getInstance().run(); 
    }
      
    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
   /*            double currentID = LimelightHelpers.getFiducialID("limelight-turret");

        if (currentID == 8) {
            // Just pass the error (tx) to the subsystem
            limelight.track(); 
          }
        if (currentID == 24) {
            // Just pass the error (tx) to the subsystem
            limelight.track(); 
          }*/
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
       /*        double currentID = LimelightHelpers.getFiducialID("limelight-turret");

        if (currentID == 10) {
            // Just pass the error (tx) to the subsystem
            limelight.track(); 
          }
        if (currentID == 26) {
            // Just pass the error (tx) to the subsystem
            limelight.track(); 
          }*/

        fan.setAccumulatorInitialValue(999999999);

    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}
