package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;

public class limelightTracking {
        private final TalonFX mechanismMotorTurret = new TalonFX(2);
        public limelightTracking() {}


        //limelight code
                //constants
            double kP = 0.0025;
            double kI = 0.0;
            double kD = 0.00025;

                //define PID
            PIDController turretPD = new PIDController(kP, kI, kD);
        
                public void track() {
            double tx = NetworkTableInstance.getDefault().getTable("limelight-turret").getEntry("tx").getDouble(0);

                //set ty to go to 0
            double motorOutput = turretPD.calculate(tx, 0);

                //apply speed with safety clamp and deadband
            if (Math.abs(tx) < 0.5) {
                mechanismMotorTurret.set(0.0);
            } else {
                mechanismMotorTurret.set(MathUtil.clamp(motorOutput, -0.1, 0.1));
            }
        }

            public void stop() {
                mechanismMotorTurret.set(0);
            }
}