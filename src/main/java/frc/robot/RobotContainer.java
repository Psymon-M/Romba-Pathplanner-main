package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Servo;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.LinearServo;
import edu.wpi.first.math.MathUtil;

public class RobotContainer {
    private final TalonFX      mechanismMotorKicker           = new TalonFX(1);
    private final TalonFX      mechanismMotorTurret           = new TalonFX(2);
    private final TalonFX      mechanismMotorLauncherforward  = new TalonFX(3);
    private final TalonFX      mechanismMotorIntake           = new TalonFX(4);
    private final TalonFX      mechanismMotorLauncherbackward = new TalonFX(5);
    private final TalonFXS     mechanismmotorminion           = new TalonFXS(6);
    public  final LinearServo  servo2                         = new LinearServo();
    private final SparkFlex vortexMotor1 = new SparkFlex(10, MotorType.kBrushless);
    
    // --- NEW LINEAR ACTUATOR ADDED HERE ---
    private final Servo myLinearActuator = new Servo(0);
    
    private final SendableChooser<Command> autoChooser;
    Orchestra                  orchestra                      = new Orchestra();
      

    
    private double MaxSpeed = 0.5 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.25).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.Idle idle = new SwerveRequest.Idle();
    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);
    private final CommandXboxController joystickOperator = new CommandXboxController(2);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

   
    public RobotContainer() {
        orchestra.loadMusic("Win.chrp");

 
        myLinearActuator.setBoundsMicroseconds(2000, 1500, 1500, 1500, 1000); 


        orchestra.addInstrument(mechanismMotorLauncherforward, 0);
        orchestra.addInstrument(mechanismMotorLauncherbackward, 0);
        

        for (int i = 0; i < 4; i++) {
            orchestra.addInstrument(drivetrain.getModule(i).getDriveMotor(), 0);
            orchestra.addInstrument(drivetrain.getModule(i).getSteerMotor(), 0);
        }
        
       
        NamedCommands.registerCommand("Shoot", shootCommand());
        NamedCommands.registerCommand("Slurp", IntakeCommand());
        NamedCommands.registerCommand("Short Shoot", ShortshootCommand());
        NamedCommands.registerCommand("Short Slurp", ShortIntakeCommand());
        
        configureBindings();
        
    
             joystickOperator.button(9).whileTrue(
            Commands.startEnd(
                () -> vortexMotor1.set(0.1),  
                () -> vortexMotor1.stopMotor() 
            )
             );        
         joystickOperator.button(12).whileTrue(
            Commands.startEnd(
                () -> vortexMotor1.set(0.1),  
                () -> vortexMotor1.stopMotor() 
            )
             );        
             joystick2.button(1).whileTrue(
            Commands.startEnd(
                () -> vortexMotor1.set(0.1),  
                () -> vortexMotor1.stopMotor() 
            )
             );    
              joystick.button(1).whileTrue(
            Commands.startEnd(
                () -> vortexMotor1.set(0.1),  
                () -> vortexMotor1.stopMotor() 
            )
             );    
             
             joystickOperator.button(2).whileTrue(
    Commands.sequence(
        Commands.waitSeconds(0.5),
         Commands.startEnd(
        () -> myLinearActuator.set(0.25),
        () -> myLinearActuator.set(0.05)
        )));   
        
        joystickOperator.button(11).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorLauncherforward.set(-0.625), 
                () -> mechanismMotorLauncherforward.stopMotor() 
            )
        );        
        
        joystickOperator.button(11).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorLauncherbackward.set(-1), 
                () -> mechanismMotorLauncherbackward.stopMotor() 
            )
        );
         
    
        joystickOperator.button(9).whileTrue(dynamicshootcommand());      
        
       
        
        
        
        
        
        joystick2.button(1).whileTrue(dynamicshootcommand());
            
       joystick2.button(1).whileTrue(
    Commands.sequence(
        Commands.startEnd(
            () -> mechanismmotorminion.set(0.1), 
            () -> mechanismmotorminion.stopMotor() 
        ).withTimeout(1)
        
    ).repeatedly() 
);
       
       
        joystickOperator.button(9).whileTrue(
    Commands.sequence(
        Commands.startEnd(
            () -> mechanismmotorminion.set(0.1), 
            () -> mechanismmotorminion.stopMotor() 
        ).withTimeout(1)
        
    ).repeatedly() 
);
           joystickOperator.button(10).whileTrue(
    Commands.sequence(
        Commands.startEnd(
            () -> mechanismmotorminion.set(0.25), 
            () -> mechanismmotorminion.stopMotor() 
        ).withTimeout(1),
        Commands.startEnd(
            () -> mechanismmotorminion.set(-0.25), 
            () -> mechanismmotorminion.stopMotor() 
        ).withTimeout(2)
        
    ).repeatedly() 
);
joystick2.button(1).whileTrue(
    Commands.sequence(
        Commands.startEnd(
            () -> mechanismmotorminion.set(1), 
            () -> mechanismmotorminion.stopMotor() 
        ).withTimeout(1),
        Commands.startEnd(
            () -> mechanismmotorminion.set(-1), 
            () -> mechanismmotorminion.stopMotor() 
        ).withTimeout(2)
        
    ).repeatedly() 
);
        
            

        joystick.button(1).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorIntake.set(0.75), 
                () -> mechanismMotorIntake.stopMotor() 
            )
        );
        
        joystickOperator.button(12).whileTrue(
            Commands.startEnd(
                () -> mechanismMotorIntake.set(1), 
                () -> mechanismMotorIntake.stopMotor() 
            )
        );
    
        joystick2.button(2).whileTrue(
            Commands.startEnd(
                () -> MaxSpeed = 0.10 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond), 
                () -> MaxSpeed = 0.15 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond)
            )
        );
     
joystickOperator.button(10).whileTrue(
    drivetrain.trackAprilTag(
        () -> -joystick.getLeftY(), 
        () -> -joystick.getLeftX()  
    )
);
        joystick2.button(3).whileTrue(
    drivetrain.trackAprilTag(
        () -> -joystick.getLeftY(), 
        () -> -joystick.getLeftX()  
    )
);
        joystick.button(3).whileTrue(
            drivetrain.applyRequest(() -> idle)
                .beforeStarting(() -> orchestra.play())
                .finallyDo(() -> orchestra.pause())
        );
joystickOperator.button(6).whileTrue(
            drivetrain.applyRequest(() -> idle)
                .beforeStarting(() -> orchestra.play())
                .finallyDo(() -> orchestra.pause())
        );
        joystickOperator.back().onTrue(Commands.runOnce(() -> orchestra.pause()));
    
        autoChooser = new SendableChooser<>();
        
       
    
        autoChooser.setDefaultOption("Default auto", defaultAutoCommand());
        autoChooser.addOption("Rizz", new PathPlannerAuto("Sigma Auto")); 
        autoChooser.addOption("half field", new PathPlannerAuto("Half field run")); 
        autoChooser.addOption("Path blue right", new PathPlannerAuto("Roomba run")); 
        autoChooser.addOption("Meter test", new PathPlannerAuto("Meter test")); 
        autoChooser.addOption("TEST", new PathPlannerAuto("Straight")); 
        autoChooser.addOption("Blue Right Score", new PathPlannerAuto("Mr scoville")); 
        autoChooser.addOption("Blue Left Score", new PathPlannerAuto("Blue Left Score")); 
        autoChooser.addOption("Do Nothing", Commands.none());
  
       
        SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    

    //Crazy? I was crazy once. They locked me in a room. A rubber room! A rubber room with rats, and rats make me crazy. 

    private void configureBindings() {
       
       
       drivetrain.setDefaultCommand(
            drivetrain.applyRequest(() ->
                drive.withVelocityX((joystick.getLeftY()) * MaxSpeed) 
                     .withVelocityY((joystick.getLeftX()) * MaxSpeed) 
                     .withRotationalRate((-joystick2.getLeftX()) * MaxAngularRate) 
            )
        );
                
        //joystick.b().whileTrue(drivetrain.applyRequest(() -> brake));
        //joystick.button(4).whileTrue(drivetrain.applyRequest(() ->
           // point.withModuleDirection(new Rotation2d(joystick.getLeftY(), joystick.getLeftX()))
       // ));

        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

         joystick.button(8).onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));


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
                mechanismMotorKicker.set(0.35);
            }),
            Commands.waitSeconds(5.0),
            Commands.runOnce(() -> {
                mechanismMotorKicker.stopMotor();
                mechanismMotorLauncherbackward.stopMotor();
                mechanismMotorLauncherforward.stopMotor();
            })
        );
    }
            
    private Command ShortshootCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                mechanismMotorLauncherforward.set(-0.75);
                mechanismMotorLauncherbackward.set(0.75);
                mechanismMotorKicker.set(0.35);
            }),
            Commands.waitSeconds(2.5),
            Commands.runOnce(() -> {
                mechanismMotorKicker.stopMotor();
                mechanismMotorLauncherbackward.stopMotor();
                mechanismMotorLauncherforward.stopMotor();
            })
        );
    }
        
    private Command IntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> mechanismMotorIntake.set(0.7)),
            Commands.waitSeconds(5.0),
            Commands.runOnce(() -> mechanismMotorIntake.stopMotor())
        );
    }

    private Command ShortIntakeCommand() {
        return Commands.sequence(
            Commands.runOnce(() -> mechanismMotorIntake.set(0.7)),
            Commands.waitSeconds(2.5),
            Commands.runOnce(() -> mechanismMotorIntake.stopMotor())
        );
    }


  private double getDistanceFromTargetInches() {
  
        double targetTagID = 26; 


        if (!LimelightHelpers.getTV("limelight-two") || LimelightHelpers.getFiducialID("limelight-two") != targetTagID) {
            return 114.4375; 
        }

        double targetHeightInches = 44.25;
        double limelightHeightInches = 21.5;
        
        double limelightMountAngleDegrees = 0.0; 

        double targetOffsetAngle_Vertical = LimelightHelpers.getTY("limelight-two");

        double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
        double angleToGoalRadians = Math.toRadians(angleToGoalDegrees);

        double distance = (targetHeightInches - limelightHeightInches) / Math.tan(angleToGoalRadians);
        return distance;
    }

    private Command dynamicshootcommand() {
        return Commands.runEnd(
            () -> {
                double distance = getDistanceFromTargetInches();
                
   
                double scaleFactor = distance / 114.4375;
                
                double forwardSpeed = -1 * scaleFactor;
                double backwardSpeed = -1 * scaleFactor;
                
                forwardSpeed = MathUtil.clamp(forwardSpeed, -0.8, 0);
                backwardSpeed = MathUtil.clamp(backwardSpeed, -0.8, 0);
                
                mechanismMotorLauncherforward.set(forwardSpeed);
                mechanismMotorLauncherbackward.set(backwardSpeed);
            },
            () -> {
                mechanismMotorLauncherforward.stopMotor();
                mechanismMotorLauncherbackward.stopMotor();
            }
        );
    }
}