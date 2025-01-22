package frc.robot.unfinished.elevator;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.controller.ProfiledPIDController;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.MathUtil;

public class AlternateElevator {
    private final SparkMax elevatorMotor;
    private final SparkMax followerMotor;
    private final XboxController controller;
    private final DigitalInput bottomLimitSwitch;
    
    // Physical constants for chain and sprocket
    private final double ENCODER_TICKS_PER_REVOLUTION = 42.0;
    private final double GEAR_RATIO = 10.0;
    private final double SPROCKET_PITCH = 0.25;
    private final double SPROCKET_TEETH = 16.0;
    private final double INCHES_PER_REVOLUTION = SPROCKET_PITCH * SPROCKET_TEETH;
    private final double TICKS_PER_INCH = (ENCODER_TICKS_PER_REVOLUTION * GEAR_RATIO) / INCHES_PER_REVOLUTION;
    
    // Motion constraints
    private final double DEFAULT_MAX_VEL = 30.0; // inches per second
    private final double DEFAULT_MAX_ACCEL = 60.0; // inches per second squared
    
    // Speed control
    private final double DEFAULT_UP_SPEED = 0.7;
    private final double DEFAULT_DOWN_SPEED = 0.5;
    private final double MIN_SPEED = 0.3;
    private final double MAX_SPEED = 1.0;
    private final double SLOW_ZONE_INCHES = 6.0; // Inches from limits to start slowing
    private final double RAMP_RATE = 0.1; // Seconds from 0 to full speed
    private double currentMaxUpSpeed = DEFAULT_UP_SPEED;
    private double currentMaxDownSpeed = DEFAULT_DOWN_SPEED;
    
    // Height presets and limits
    private final double MAX_HEIGHT_INCHES = 48.0;
    private double[] presetHeights = {
        0.0,   // GROUND
        12.0,  // LOW
        24.0,  // MID
        36.0,  // HIGH
        48.0   // MAX
    };
    private double[] presetSpeeds = {
        0.5,   // GROUND speed
        0.6,   // LOW speed
        0.7,   // MID speed
        0.6,   // HIGH speed
        0.5    // MAX speed
    };
    
    // Controllers
    private final ProfiledPIDController profiledPIDController;
    private final TrapezoidProfile.Constraints constraints;
    
    // State variables
    private boolean isManualControl = false;
    private boolean isBottomSeeking = false;
    private double targetPosition = 0.0;
    @SuppressWarnings("unused")
    private double lastPosition = 0.0;
    
    public AlternateElevator() {
        elevatorMotor = new SparkMax(1, MotorType.kBrushless);
        followerMotor = new SparkMax(1, MotorType.kBrushless);
        controller = new XboxController(0);
        bottomLimitSwitch = new DigitalInput(0);
        
        SparkMaxConfig followConfig = new SparkMaxConfig();
        followConfig.follow(elevatorMotor);
        followerMotor.configure(followConfig, null, null);

        SparkMaxConfig elevatorConfig = new SparkMaxConfig();
        elevatorConfig.idleMode(IdleMode.kBrake);
        elevatorConfig.smartCurrentLimit(40);
        elevatorConfig.openLoopRampRate(RAMP_RATE);
        elevatorMotor.getEncoder().setPosition(0);
        
        // Configure motion profiling
        constraints = new TrapezoidProfile.Constraints(DEFAULT_MAX_VEL, DEFAULT_MAX_ACCEL);
        profiledPIDController = new ProfiledPIDController(1.0, 0.0, 0.0, constraints);
        profiledPIDController.setTolerance(0.25);
        
        initializeSmartDashboard();
    }
    
    private void initializeSmartDashboard() {
        SmartDashboard.putNumber("Max Up Speed", DEFAULT_UP_SPEED);
        SmartDashboard.putNumber("Max Down Speed", DEFAULT_DOWN_SPEED);
        SmartDashboard.putNumber("Max Velocity (in/s)", DEFAULT_MAX_VEL);
        SmartDashboard.putNumber("Max Acceleration (in/s²)", DEFAULT_MAX_ACCEL);
        for (int i = 0; i < presetHeights.length; i++) {
            SmartDashboard.putNumber("Preset " + i + " Height", presetHeights[i]);
            SmartDashboard.putNumber("Preset " + i + " Speed", presetSpeeds[i]);
        }
    }
    
    private double ticksToInches(double ticks) {
        return ticks / TICKS_PER_INCH;
    }
    
    private double getCurrentHeightInches() {
        return ticksToInches(elevatorMotor.getEncoder().getPosition());
    }
    
    private double getPresetSpeed(double targetHeight) {
        // Find closest preset and use its speed
        double closestDistance = Double.MAX_VALUE;
        double speed = DEFAULT_UP_SPEED;
        
        for (int i = 0; i < presetHeights.length; i++) {
            double distance = Math.abs(presetHeights[i] - targetHeight);
            if (distance < closestDistance) {
                closestDistance = distance;
                speed = presetSpeeds[i];
            }
        }
        return speed;
    }
    
    private double applySpeedLimits(double requestedSpeed) {
        double currentHeight = getCurrentHeightInches();
        
        // Update speed limits from dashboard
        currentMaxUpSpeed = MathUtil.clamp(
            SmartDashboard.getNumber("Max Up Speed", DEFAULT_UP_SPEED),
            MIN_SPEED, MAX_SPEED
        );
        currentMaxDownSpeed = MathUtil.clamp(
            SmartDashboard.getNumber("Max Down Speed", DEFAULT_DOWN_SPEED),
            MIN_SPEED, MAX_SPEED
        );
        
        // Determine base speed limit based on direction
        double speedLimit = (requestedSpeed > 0) ? currentMaxUpSpeed : currentMaxDownSpeed;
        
        // Reduce speed near limits
        if (currentHeight < SLOW_ZONE_INCHES) {
            speedLimit *= currentHeight / SLOW_ZONE_INCHES;
        } else if (currentHeight > (MAX_HEIGHT_INCHES - SLOW_ZONE_INCHES)) {
            speedLimit *= (MAX_HEIGHT_INCHES - currentHeight) / SLOW_ZONE_INCHES;
        }
        
        // Apply preset-specific speed limit
        double presetSpeedLimit = getPresetSpeed(targetPosition);
        speedLimit = Math.min(speedLimit, presetSpeedLimit);
        
        // Clamp final speed
        return MathUtil.clamp(requestedSpeed, -speedLimit, speedLimit);
    }
    
    public void periodic() {
        // Update preset heights from dashboard
        for (int i = 0; i < presetHeights.length; i++) {
            presetHeights[i] = SmartDashboard.getNumber("Preset " + i + " Height", presetHeights[i]);
            presetSpeeds[i] = SmartDashboard.getNumber("Preset " + i + " Speed", presetSpeeds[i]);
        }
        
        // Handle bottom limit switch
        if (bottomLimitSwitch.get()) {
            elevatorMotor.getEncoder().setPosition(0);
            if (isBottomSeeking) {
                elevatorMotor.set(0);
                isBottomSeeking = false;
                isManualControl = false;
                targetPosition = 0;
                profiledPIDController.reset(0);
            }
        }
        
        // Handle bottom seeking (O/B button)
        if (controller.getBButtonPressed()) {
            isBottomSeeking = true;
            isManualControl = true;
        }
        
        if (isBottomSeeking) {
            elevatorMotor.set(applySpeedLimits(-0.15));
            return;
        }
        
        // Handle preset commands
        if (controller.getPOV() != -1 || controller.getYButton()) {
            isManualControl = false;
            if (controller.getPOV() == 0) targetPosition = presetHeights[4];         // Max
            else if (controller.getPOV() == 90) targetPosition = presetHeights[3];   // High
            else if (controller.getPOV() == 180) targetPosition = presetHeights[0];  // Ground
            else if (controller.getPOV() == 270) targetPosition = presetHeights[1];  // Low
            else if (controller.getYButton()) targetPosition = presetHeights[2];     // Mid
            profiledPIDController.reset(getCurrentHeightInches());
        }
        
        // Handle manual control
        if (controller.getLeftBumper() || controller.getRightBumper()) {
            isManualControl = true;
            double adjustment = 0;
            if (controller.getLeftBumper() && !bottomLimitSwitch.get()) adjustment = -0.3;
            if (controller.getRightBumper()) adjustment = 0.3;
            elevatorMotor.set(applySpeedLimits(adjustment));
            lastPosition = getCurrentHeightInches();
        } 
        // Handle profiled movement
        else if (!isManualControl) {
            double currentHeight = getCurrentHeightInches();
            double speed = profiledPIDController.calculate(currentHeight, targetPosition);
            speed = applySpeedLimits(speed);
            
            if (bottomLimitSwitch.get() && speed < 0) {
                speed = 0;
                profiledPIDController.reset(0);
            }
            
            elevatorMotor.set(speed);
        }
        
        // Emergency stop
        if (controller.getStartButton()) {
            elevatorMotor.set(0);
            isManualControl = true;
            profiledPIDController.reset(getCurrentHeightInches());
        }
        
        // Update dashboard
        updateSmartDashboard();
    }
    
    private void updateSmartDashboard() {
        SmartDashboard.putNumber("Current Height (inches)", getCurrentHeightInches());
        SmartDashboard.putNumber("Target Height (inches)", targetPosition);
        SmartDashboard.putNumber("Current Speed", elevatorMotor.get());
        SmartDashboard.putNumber("Applied Speed Limit", Math.abs(elevatorMotor.get()));
        SmartDashboard.putBoolean("At Target", profiledPIDController.atSetpoint());
        SmartDashboard.putBoolean("Manual Control", isManualControl);
        SmartDashboard.putBoolean("Bottom Limit", bottomLimitSwitch.get());
        SmartDashboard.putBoolean("Seeking Bottom", isBottomSeeking);
    }
}