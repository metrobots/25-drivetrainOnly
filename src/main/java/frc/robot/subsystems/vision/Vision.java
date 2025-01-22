package frc.robot.subsystems.vision;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.utils.LimelightLib;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Vision extends SubsystemBase {
    private final Drivetrain drivetrain;
    
    private final PIDController turnPIDController;
    private final PIDController distancePIDController;
    private final PIDController strafePIDController;
    
    // PID Constants
    private static final double TURN_P = 0.05;
    private static final double TURN_I = 0.0;
    private static final double TURN_D = 0.0;
    
    private static final double DISTANCE_P = 0.03;
    private static final double DISTANCE_I = 0.0;
    private static final double DISTANCE_D = 0.0;
    
    private static final double STRAFE_P = 0.04;
    private static final double STRAFE_I = 0.0;
    private static final double STRAFE_D = 0.0;
    
    private static final double TURN_TOLERANCE_DEGREES = 2.0;
    private static final double DISTANCE_TOLERANCE_INCHES = 2.0;
    private static final double STRAFE_TOLERANCE_DEGREES = 2.0;

    private static final String name = "front";
    
    public Vision(Drivetrain drivetrain) {
        this.drivetrain = drivetrain;
        
        turnPIDController = new PIDController(TURN_P, TURN_I, TURN_D);
        distancePIDController = new PIDController(DISTANCE_P, DISTANCE_I, DISTANCE_D);
        strafePIDController = new PIDController(STRAFE_P, STRAFE_I, STRAFE_D);
        
        turnPIDController.setTolerance(TURN_TOLERANCE_DEGREES);
        distancePIDController.setTolerance(DISTANCE_TOLERANCE_INCHES);
        strafePIDController.setTolerance(STRAFE_TOLERANCE_DEGREES);
    }
    
    @Override
    public void periodic() {
        updateSmartDashboard();
    }
    
    /**
     * Get forward/backward correction (X axis)
     * @param targetDistanceInches desired distance from target
     * @return correction value (-1 to 1)
     */
    public double getForwardCorrection(double targetDistanceInches) {
        if (!hasTarget()) return 0.0;
        
        double currentDistance = getDistanceToTarget();
        double correction = distancePIDController.calculate(currentDistance, targetDistanceInches);
        
        return -Math.min(Math.max(correction, -0.5), 0.5); // Negative because forward is negative
    }
    
    /**
     * Get side-to-side correction (Y axis)
     * @param targetX desired X offset (usually 0 to center)
     * @return correction value (-1 to 1)
     */
    public double getStrafeCorrection(double targetX) {
        if (!hasTarget()) return 0.0;
        
        double currentX = LimelightLib.getTX(name);
        double correction = strafePIDController.calculate(currentX, targetX);
        
        return Math.min(Math.max(correction, -0.5), 0.5);
    }
    
    /**
     * Get rotation correction
     * @param targetAngle desired angle to target
     * @return correction value (-1 to 1)
     */
    public double getRotationCorrection(double targetAngle) {
        if (!hasTarget()) return 0.0;
        
        double currentAngle = LimelightLib.getTX(name);
        double correction = turnPIDController.calculate(currentAngle, targetAngle);
        
        return Math.min(Math.max(correction, -0.5), 0.5);
    }
    
    /**
     * Check if forward/backward movement is at target
     */
    public boolean atDistanceTarget() {
        return distancePIDController.atSetpoint();
    }
    
    /**
     * Check if side-to-side movement is at target
     */
    public boolean atStrafeTarget() {
        return strafePIDController.atSetpoint();
    }
    
    /**
     * Check if rotation is at target
     */
    public boolean atRotationTarget() {
        return turnPIDController.atSetpoint();
    }
    
    /**
     * Get distance to target using Limelight's pose estimation
     * @return distance in inches
     */
    public double getDistanceToTarget() {
        double[] pose = LimelightLib.getTargetPose_CameraSpace(name);
        return pose[2];  // Z component represents forward distance
    }
    
    /**
     * Example of how to combine all corrections
     * @param targetDistance desired distance in inches
     * @param targetX desired X offset
     * @param targetAngle desired angle
     */
    public void alignToTarget(double targetDistance, double targetX, double targetAngle) {
        if (!hasTarget()) {
            drivetrain.drive(0, 0, 0, true);
            return;
        }
        
        double xSpeed = getForwardCorrection(targetDistance);
        double ySpeed = getStrafeCorrection(targetX);
        double rotationSpeed = getRotationCorrection(targetAngle);
        
        drivetrain.drive(xSpeed, ySpeed, rotationSpeed, true);
    }
    
    /**
     * Check if target is visible
     * @return true if target is visible, false otherwise
     */
    public boolean hasTarget() {
        return LimelightLib.getTV(name);
    }
    
    /**
     * Update SmartDashboard with vision data
     */
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Limelight X", LimelightLib.getTX(name));
        SmartDashboard.putNumber("Limelight Y", LimelightLib.getTY(name));
        SmartDashboard.putNumber("Distance to Target (inches)", getDistanceToTarget());
        SmartDashboard.putBoolean("Has Target", hasTarget());
    }
    
    /**
     * Set Limelight LED mode
     * @param on true to turn on LEDs, false to turn off
     */
    public void setLEDs(boolean on) {
        if (on) {
            LimelightLib.setLEDMode_ForceOn(name);
        }
    }
}

/* DIFFERENT WAYS TO USE THIS FILE

// Use individual corrections
double forward = vision.getForwardCorrection(36.0);  // Move to 36 inches
double strafe = vision.getStrafeCorrection(0.0);     // Center horizontally
double rotation = vision.getRotationCorrection(0.0);  // Face the target

drivetrain.drive(forward, strafe, rotation, true);

// Or use the combined alignment
vision.alignToTarget(36.0, 0.0, 0.0);

// Check if aligned
boolean isAligned = vision.atDistanceTarget() && 
                   vision.atStrafeTarget() && 
                   vision.atRotationTarget(); 
                   
*/