// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix6.hardware.TalonFX;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    private final TalonFX mechanismMotorKicker = new TalonFX(1);
    private final TalonFX mechanismMotorTurret = new TalonFX(2);
    private final TalonFX mechanismMotorLauncherforward = new TalonFX(3);
    private final TalonFX mechanismMotorIntake = new TalonFX(4);
    private final TalonFX mechanismMotorLauncherbackward = new TalonFX(5);
    private final DigitalInput myLimitSwitch = new DigitalInput(2);
    private final Servo mySmartServo = new Servo(9);

   private final SendableChooser<Command> autoChooser;
    
    private double MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.50).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.2).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
  private final CommandXboxController joystick2 = new CommandXboxController(1);
  private final CommandXboxController joystickOperator = new CommandXboxController(2);
    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

   

    public RobotContainer() {

    NamedCommands.registerCommand("Shoot", shootCommand());
    NamedCommands.registerCommand("Slurp", IntakeCommand());
    NamedCommands.registerCommand("Short Shoot", ShortshootCommand());
    NamedCommands.registerCommand("Short Slurp", ShortIntakeCommand());
        
        configureBindings();
        joystickOperator.leftTrigger()
            .and(() -> myLimitSwitch.get() == true) 
            .whileTrue(Commands.startEnd(
                () -> mechanismMotorTurret.set(0.05), 
                () -> mechanismMotorTurret.stopMotor() 
            )
        );
        joystickOperator.rightTrigger()
            .whileTrue(Commands.startEnd(
                () -> mechanismMotorTurret.set(-0.05), 
                () -> mechanismMotorTurret.stopMotor() 
            )
        );
            
           joystickOperator.button(1).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorLauncherforward.set(-0.75), 
                () -> mechanismMotorLauncherforward.stopMotor() 
            )
        );        
          joystickOperator.button(1).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorLauncherbackward.set(0.75), 
                () -> mechanismMotorLauncherbackward.stopMotor() 
            )
        );
                 joystickOperator.button(2).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorLauncherforward.set(-0.75), 
                () -> mechanismMotorLauncherforward.stopMotor() 
            )
        );        
          joystickOperator.button(2).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorLauncherbackward.set(0.75), 
                () -> mechanismMotorLauncherbackward.stopMotor() 
            )
        );
           joystickOperator.button(1).whileTrue(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.startEnd(
                () -> mechanismMotorKicker.set(0.35), 
                () -> mechanismMotorKicker.stopMotor())
)
        );
         joystickOperator.button(2).whileTrue(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.startEnd(
                () -> mechanismMotorKicker.set(0.35), 
                () -> mechanismMotorKicker.stopMotor())
)
        );
        
   joystickOperator.button(3).whileTrue(
                Commands.sequence(
                    Commands.waitSeconds(0.5),
                    Commands.startEnd(
                () -> mechanismMotorKicker.set(-0.35), 
                () -> mechanismMotorKicker.stopMotor())
            )
        );


        joystickOperator.rightBumper().whileTrue(
        Commands.startEnd(
                () -> mechanismMotorIntake.set(-0.55), 
                () -> mechanismMotorIntake.stopMotor() 
            )
        );
       joystickOperator.button(2).whileTrue(
            Commands.startEnd(
                () -> mySmartServo.set(-10.0),
                () -> mySmartServo.set(0.5)

            )
        );
    joystickOperator.button(1).whileTrue(
            Commands.startEnd(
                () -> mySmartServo.set(-90.0),
                () -> mySmartServo.set(0.5)

            )
        );
     joystick2.button(1).whileTrue(
            
        Commands.startEnd(
                () -> mechanismMotorIntake.set(-0.75), 
                () -> mechanismMotorIntake.stopMotor() 
            )
        );
    joystick2.button(2).whileTrue(
            Commands.startEnd(
                () -> MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 
                () -> MaxSpeed = 0.75 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            )
        );
    autoChooser = new SendableChooser<>();
    
    autoChooser.setDefaultOption("Default auto", defaultAutoCommand());
    autoChooser.addOption("Rizz", new PathPlannerAuto("Sigma Auto")); 
    autoChooser.addOption("half field", new PathPlannerAuto("Half field run")); 
     autoChooser.addOption("Path blue right", new PathPlannerAuto("Roomba run")); 
   autoChooser.addOption("Meter test", new PathPlannerAuto("Meter test")); 
     autoChooser.addOption("TEST", new PathPlannerAuto("Straight")); 
   autoChooser.addOption("Do Nothing", Commands.none());
  
       
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }
    

    //Crazy? I was crazy once. They locked me in a room. A rubber room! A rubber room with rats, and rats make me crazy. 

    private void configureBindings() {
       final double joystickDeadband = 0.1;

   
       final double joystick2Deadband = 0.1;
       drivetrain.setDefaultCommand(
        
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-MathUtil.applyDeadband(joystick.getLeftY(), joystickDeadband) * MaxSpeed) 
                     .withVelocityY(-MathUtil.applyDeadband(joystick.getLeftX(), joystickDeadband) * MaxSpeed) 
                     .withRotationalRate(-MathUtil.applyDeadband(joystick2.getLeftX(), joystick2Deadband) * MaxAngularRate) 
           
                     )
        );
                    

        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

    

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

    joystick.button(1).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

  
    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private Command defaultAutoCommand() {
        return Commands.sequence(
            shootCommand()
        );
    }
   
    private Command shootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                mechanismMotorLauncherforward.set(-0.75);
                mechanismMotorLauncherbackward.set(0.75);
                mySmartServo.set(-90.0);
                mechanismMotorKicker.set(0.35);
            }),
            Commands.waitSeconds(5),
            Commands.runOnce(() -> {
                mechanismMotorKicker.stopMotor();
                mechanismMotorLauncherbackward.stopMotor();
                mechanismMotorLauncherforward.stopMotor();
                mySmartServo.set(0.5);
            }));}
           
           private Command ShortshootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                mechanismMotorLauncherforward.set(-0.75);
                mechanismMotorLauncherbackward.set(0.75);
                mySmartServo.set(-90.0);
                mechanismMotorKicker.set(0.35);
            }),
            Commands.waitSeconds(2.5),
            Commands.runOnce(() -> {
                mechanismMotorKicker.stopMotor();
                mechanismMotorLauncherbackward.stopMotor();
                mechanismMotorLauncherforward.stopMotor();
                mySmartServo.set(0.5);
            }));}
        
   
   
    private Command IntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                mechanismMotorIntake.set(0.7);
                Commands.waitSeconds(5);
                mechanismMotorIntake.stopMotor();
            })
            );
    }

      private Command ShortIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                mechanismMotorIntake.set(0.7);
                Commands.waitSeconds(2.5);
                mechanismMotorIntake.stopMotor();
            })
            );
    }
}
    
    