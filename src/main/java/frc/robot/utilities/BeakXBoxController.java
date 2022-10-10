// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utilities;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

public final class BeakXBoxController {
    public static final double THUMBSTICK_DEADBAND = 0.05; // Jiggle room for the thumbsticks
    public static final double THUMBSTICK_SENSITIVITY = 0.05;
    public static final double TRIGGER_DEADBAND = 0.01; // Jiggle room for the triggers
    public static final double TRIGGER_SENSITIVITY = 0.6; // If the trigger is beyond this limit, say it has been
                                                          // pressed

    /* Button Mappings */
    public static final class Buttons {
        public static final int A = 1;
        public static final int B = 2;
        public static final int X = 3;
        public static final int Y = 4;
        public static final int LEFT_BUMPER = 5;
        public static final int RIGHT_BUMPER = 6;
        public static final int BACK = 7;
        public static final int START = 8;
        public static final int LEFT_STICK = 9;
        public static final int RIGHT_STICK = 10;
    }

    /* Axis Mappings */
    public static final class Axes {
        public static final int LEFT_X = 0;
        public static final int LEFT_Y = 1;
        public static final int LEFT_TRIGGER = 2;
        public static final int RIGHT_TRIGGER = 3;
        public static final int RIGHT_X = 4;
        public static final int RIGHT_Y = 5;
    }

    private XboxController controller;

    public Thumbstick left_stick;
    public Thumbstick right_stick;

    public JoystickButton a;
    public JoystickButton b;
    public JoystickButton x;
    public JoystickButton y;
    public JoystickButton start;
    public JoystickButton back;
    public JoystickButton lb;
    public JoystickButton rb;
    public JoystickButton ls;
    public JoystickButton rs;

    public Trigger lt;
    public Trigger rt;

    private int port;

    /**
     * Wrapper around an XboxController with additional
     * functionality, like deadbands, individual Button
     * control, Trigger control, rumble, etc.
     */
    public BeakXBoxController(int port) {
        controller = new XboxController(port);
        this.port = port;

        left_stick = new Thumbstick(controller, HAND.LEFT);
        right_stick = new Thumbstick(controller, HAND.RIGHT);

        a = new JoystickButton(controller, Buttons.A);
        b = new JoystickButton(controller, Buttons.B);
        x = new JoystickButton(controller, Buttons.X);
        y = new JoystickButton(controller, Buttons.Y);
        lb = new JoystickButton(controller, Buttons.LEFT_BUMPER);
        rb = new JoystickButton(controller, Buttons.RIGHT_BUMPER);
        back = new JoystickButton(controller, Buttons.BACK);
        start = new JoystickButton(controller, Buttons.START);
        ls = new JoystickButton(controller, Buttons.LEFT_STICK);
        rs = new JoystickButton(controller, Buttons.RIGHT_STICK);

        lt = new Trigger(controller, HAND.LEFT);
        rt = new Trigger(controller, HAND.RIGHT);
    }

    public double getLeftXAxis() {
        return left_stick.getX();
    }

    public double getLeftYAxis() {
        return left_stick.getY();
    }

    public double getRightXAxis() {
        return right_stick.getX();

    }

    public double getRightYAxis() {
        return right_stick.getY();
    }

    public double getLeftTrigger() {
        return lt.getX();
    }

    public double getRightTrigger() {
        return rt.getX();
    }

    public static enum HAND {
        LEFT, RIGHT
    }

    public static class Thumbstick extends Button {
        /* Instance Values */
        private final XboxController parent;
        private final HAND hand;
        private final int xAxisID;
        private final int yAxisID;
        private final int pressedID;
        private double xDeadZone;
        private double yDeadZone;
        private double sensitivity;

        /**
         * Constructor
         * 
         * @param parent
         * @param hand
         */
        public Thumbstick(final XboxController parent, final HAND hand) {
            /* Initialize */
            this.parent = parent;
            this.hand = hand;
            this.xDeadZone = THUMBSTICK_DEADBAND;
            this.yDeadZone = THUMBSTICK_DEADBAND;
            this.sensitivity = THUMBSTICK_SENSITIVITY;

            if (hand == HAND.LEFT) {
                this.xAxisID = Axes.LEFT_X;
                this.yAxisID = Axes.LEFT_Y;
                this.pressedID = Buttons.LEFT_STICK;
            } else { // If right hand
                this.xAxisID = Axes.RIGHT_X;
                this.yAxisID = Axes.RIGHT_Y;
                this.pressedID = Buttons.RIGHT_STICK;
            }
        }

        /**
         * + = right
         * - = left
         * 
         * @return X but with a deadzone
         */
        private double rawX() {
            final double rawInput = parent.getRawAxis(xAxisID);
            return createDeadZone(rawInput, xDeadZone);
        }

        /**
         * + = up
         * - = down
         * 
         * @return Y but with a deadzone
         */
        private double rawY() {
            final double rawInput = -parent.getRawAxis(yAxisID); // -Y was up on our thumbsticks. Consider this a fix?
            return createDeadZone(rawInput, yDeadZone);
        }

        /**
         * magnitude
         * 
         * @param x
         * @param y
         * @return Magnitude of thing
         */
        private double magnitude(double x, double y) {
            final double xSquared = Math.pow(x, 2);
            final double ySquared = Math.pow(y, 2);
            return Math.sqrt(xSquared + ySquared);
        }

        /**
         * angleToSquareSpace
         * 
         * @param angle
         * @return Number between 0 and PI/4
         */
        private double angleToSquareSpace(double angle) {
            final double absAngle = Math.abs(angle);
            final double halfPi = Math.PI / 2;
            final double quarterPi = Math.PI / 4;
            final double modulus = absAngle % halfPi;

            return -Math.abs(modulus - quarterPi) + quarterPi;
        }

        /**
         * scaleMagnitude
         * 
         * @param x
         * @param y
         * @return
         */
        private double scaleMagnitude(double x, double y) {
            final double magnitude = magnitude(x, y);
            final double angle = Math.atan2(x, y);
            final double newAngle = angleToSquareSpace(angle);
            final double scaleFactor = Math.cos(newAngle);

            return magnitude * scaleFactor;
        }

        /* Extended Methods */
        @Override
        public boolean get() {
            return Math.abs(getX()) > sensitivity || Math.abs(getY()) > sensitivity;
        }

        /* Get Methods */
        /**
         * Used to see which side of the controller this thumbstick is
         * 
         * @return Thumbstick hand
         */
        public HAND getHand() {
            return hand;
        }

        /**
         * getRawX
         * 
         * @return X with a deadzone
         */
        public double getX() {
            return rawX();
        }

        /**
         * getRawY
         * 
         * @return Y with a deadzone
         */
        public double getY() {
            return rawY();
        }

        /**
         * 0 = Up;
         * 90 = Right;
         * ...
         * 
         * @return Angle the thumbstick is pointing
         */
        public double getAngle() {
            final double angle = Math.atan2(rawX(), rawY());
            return Math.toDegrees(angle);
        }

        /**
         * getMagnitude
         * 
         * @return A number between 0 and 1
         */
        public double getMagnitude() {
            double magnitude = scaleMagnitude(rawX(), rawY());

            if (magnitude > 1) {
                magnitude = 1; // Prevent any errors that might arise
            }

            return magnitude;
        }

        /**
         * Get the adjusted thumbstick position (Magnitude at most 1)
         * 
         * @return True thumbstick position
         */
        public double getTrueX() {
            final double x = rawX();
            final double y = rawY();
            final double angle = Math.atan2(x, y);

            return scaleMagnitude(x, y) * Math.sin(angle);
        }

        /**
         * Get the adjusted thumbstick position (Magnitude at most 1)
         * 
         * @return True thumbstick position
         */
        public double getTrueY() {
            final double x = rawX();
            final double y = rawY();
            final double angle = Math.atan2(x, y);

            return scaleMagnitude(x, y) * Math.cos(angle);
        }

        /* Set Methods */
        /**
         * Set the X axis deadzone of this thumbstick
         * 
         * @param number
         */
        public void setXDeadZone(double number) {
            xDeadZone = number;
        }

        /**
         * Set the Y axis deadzone of this thumbstick
         * 
         * @param number
         */
        public void setYDeadZone(double number) {
            yDeadZone = number;
        }

        /**
         * Set both axis deadzones of this thumbstick
         * 
         * @param number
         */
        public void setDeadZone(double number) {
            xDeadZone = number;
            yDeadZone = number;
        }

        public boolean getIsJoystickHeld() {
            return parent.getRawButton(pressedID);
        }

        public boolean getIsJoystickPressed() {
            return parent.getRawButtonPressed(pressedID);
        }

        public boolean getIsJoystickReleased() {
            return parent.getRawButtonReleased(pressedID);
        }
    }

    public static class Trigger extends Button {
        /* Instance Values */
        private final XboxController parent;
        private final HAND hand;

        private double deadZone;
        private double sensitivity;

        /**
         * Constructor
         * 
         * @param joystick
         * @param hand
         */
        public Trigger(final XboxController joystick, final HAND hand) {
            /* Initialize */
            this.parent = joystick;
            this.hand = hand;
            this.deadZone = TRIGGER_DEADBAND;
            this.sensitivity = TRIGGER_SENSITIVITY;
        }

        /* Extended Methods */
        @Override
        public boolean get() {
            return getX() > sensitivity;
        }

        /* Get Methods */
        /**
         * getHand
         * 
         * @return Trigger hand
         * 
         *         See which side of the controller this trigger is
         */
        public HAND getHand() {
            return hand;
        }

        /**
         * 0 = Not pressed
         * 1 = Completely pressed
         * 
         * @return How far its pressed
         */
        public double getX() {
            final double rawInput;

            if (hand == HAND.LEFT) {
                rawInput = parent.getRawAxis(2);
            } else {
                rawInput = parent.getRawAxis(3);
            }

            return createDeadZone(rawInput, deadZone);
        }

        public double getY() {
            return getX(); // Triggers have one dimensional movement. Use getX() instead
        }

        /**
         * How far you need to press this trigger to activate a button press
         * 
         * @param number
         */
        public void setTriggerSensitivity(double number) {
            this.sensitivity = number;
        }
    }

    /**
     * Creates a deadzone, but without clipping the lower values.
     * turns this
     * |--1--2--3--4--5--|
     * into this
     * ______|-1-2-3-4-5-|
     * 
     * @param input
     * @param deadZoneSize
     * @return adjusted_input
     */
    private static double createDeadZone(double input, double deadZoneSize) {
        final double negative;
        double deadZoneSizeClamp = deadZoneSize;
        double adjusted;

        if (deadZoneSizeClamp < 0 || deadZoneSizeClamp >= 1) {
            deadZoneSizeClamp = 0; // Prevent any weird errors
        }

        negative = input < 0 ? -1 : 1;

        adjusted = Math.abs(input) - deadZoneSizeClamp; // Subtract the deadzone from the magnitude
        adjusted = adjusted < 0 ? 0 : adjusted; // if the new input is negative, make it zero
        adjusted = adjusted / (1 - deadZoneSizeClamp); // Adjust the adjustment so it can max at 1

        return negative * adjusted;
    }

    /* Get Methods */
    /**
     * @return The port of this XboxController
     */
    public int getPort() {
        return port;
    }

    /**
     * @return The Joystick of this XboxController
     */
    public XboxController getJoystick() {
        return controller;
    }

    /* Set Methods */
    /**
     * Make the controller vibrate
     * 
     * @param hand      The side of the controller to rumble
     * @param intensity How strong the rumble is
     */
    public void setRumble(HAND hand, double intensity) {
        final double amount = intensity;

        if (hand == HAND.LEFT) {
            controller.setRumble(RumbleType.kLeftRumble, amount);
        } else {
            controller.setRumble(RumbleType.kRightRumble, amount);
        }
    }

    /**
     * Make the controller vibrate
     * 
     * @param intensity How strong the rumble is
     */
    public void setRumble(double intensity) {
        final double amount = intensity;

        controller.setRumble(RumbleType.kLeftRumble, amount);
        controller.setRumble(RumbleType.kRightRumble, amount);
    }

    /*
     * Set both axis deadzones of both thumbsticks
     * 
     * @param number
     */
    public void setDeadZone(double number) {
        left_stick.setDeadZone(number);
        right_stick.setDeadZone(number);
    }
}