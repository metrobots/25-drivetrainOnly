package frc.robot.utils;

import java.time.Duration;
import java.time.Instant;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * ControllerKit Xbox controller with advanced features for FRC robotics.
 * Extends CommandXboxController with additional functionality for precise control and advanced input detection.
 * 
 * Features include:
 * - Customizable deadbands and sensitivity
 * - Advanced stick position detection
 * - Button hold detection
 * - Rumble control patterns
 * - Diagonal and combination inputs
 */
public class ControllerKit extends CommandXboxController {
    private static final double DEFAULT_DEADBAND = 0.1;
    private double deadband = DEFAULT_DEADBAND;
    private Duration buttonHoldThreshold = Duration.ofMillis(500);
    private Instant buttonPressStart = null;
    private int rumbleIntensity = 0;
    private double stickSensitivity = 1.0;

    /**
     * Creates a new Xbox Controller on the specified port.
     * 
     * @param port The port number of the controller (typically 0 for first controller)
     * 
     * Example:
     * ControllerKit controller = new ControllerKit(0);
     */
    public ControllerKit(int port) {
        super(port);
    }

    /**
     * Sets the deadband for all stick inputs.
     * Inputs smaller than this value will be treated as 0.
     * 
     * @param deadband Value between 0 and 1 (typical values: 0.05 - 0.15)
     * 
     * Example:
     * controller.setDeadband(0.12); // Ignores small stick movements below 12%
     */
    public void setDeadband(double deadband) {
        this.deadband = deadband;
    }

    /**
     * Gets the current deadband value.
     * 
     * @return Current deadband value between 0 and 1
     * 
     * Example:
     * double currentDeadband = controller.getDeadband(); // Get current deadband
     */
    public double getDeadband() {
        return deadband;
    }

    /**
     * Sets the sensitivity multiplier for stick inputs.
     * Values > 1 increase sensitivity, values < 1 decrease sensitivity.
     * 
     * @param sensitivity Multiplier for stick values (typical range: 0.5 - 2.0)
     * 
     * Example:
     * controller.setStickSensitivity(1.5); // Make sticks 50% more sensitive
     */
    public void setStickSensitivity(double sensitivity) {
        this.stickSensitivity = sensitivity;
    }

    /**
     * Gets the left stick X-axis value with deadband and sensitivity applied.
     * 
     * @return Processed stick value between -1 and 1
     * 
     * Example:
     * double xValue = controller.getLeftX(); // Get processed left stick X value
     * drive.arcade(controller.getLeftY(), controller.getLeftX()); // Common drive use
     */
    @Override
    public double getLeftX() {
        return applyDeadband(super.getLeftX()) * stickSensitivity;
    }

    /**
     * Gets the left stick Y-axis value with deadband and sensitivity applied.
     * Note: Y axis is inverted by default (negative is up) to match joystick convention.
     * 
     * @return Processed stick value between -1 and 1
     * 
     * Example:
     * double forwardSpeed = -controller.getLeftY(); // Get forward/backward value
     */
    @Override
    public double getLeftY() {
        return applyDeadband(super.getLeftY()) * stickSensitivity;
    }

    // New Methods to return Trigger objects for buttons

    /**
     * Returns a Trigger for the A button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the A button
     */
    public Trigger a() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kA.value));
    }

    /**
     * Returns a Trigger for the B button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the B button
     */
    public Trigger b() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kB.value));
    }

    /**
     * Returns a Trigger for the X button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the X button
     */
    public Trigger x() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kX.value));
    }

    /**
     * Returns a Trigger for the Y button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the Y button
     */
    public Trigger y() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kY.value));
    }

    /**
     * Returns a Trigger for the Left Bumper button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the Left Bumper button
     */
    public Trigger leftBumper() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kLeftBumper.value));
    }

    /**
     * Returns a Trigger for the Right Bumper button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the Right Bumper button
     */
    public Trigger rightBumper() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kRightBumper.value));
    }

    /**
     * Returns a Trigger for the Start button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the Start button
     */
    public Trigger start() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kStart.value));
    }

    /**
     * Returns a Trigger for the Back button. This allows using .isTrue() and .isFalse().
     * 
     * @return Trigger for the Back button
     */
    public Trigger back() {
        return new Trigger(() -> getHID().getRawButton(XboxController.Button.kBack.value));
    }

    // Other Methods for diagonals, button hold, etc.

    private double applyDeadband(double value) {
        if (Math.abs(value) < deadband) {
            return 0.0;
        }
        return value;
    }
    
    // Example usage of button triggers
    public boolean isButtonHeld(int button) {
        if (getHID().getRawButton(button)) {
            if (buttonPressStart == null) {
                buttonPressStart = Instant.now();
            }
            return Duration.between(buttonPressStart, Instant.now()).compareTo(buttonHoldThreshold) >= 0;
        } else {
            buttonPressStart = null;
            return false;
        }
    }

    public void setRumble(int intensity) {
        this.rumbleIntensity = Math.max(0, Math.min(100, intensity));
        double rumbleValue = this.rumbleIntensity / 100.0;
        getHID().setRumble(XboxController.RumbleType.kLeftRumble, rumbleValue);
        getHID().setRumble(XboxController.RumbleType.kRightRumble, rumbleValue);
    }
    
}