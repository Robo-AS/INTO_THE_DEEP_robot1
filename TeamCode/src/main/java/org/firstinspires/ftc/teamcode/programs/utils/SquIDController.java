package org.firstinspires.ftc.teamcode.programs.utils;

public class SquIDController {
    private double squP;

    private boolean continuous;
    private double maximumInput;
    private double minimumInput;

    public SquIDController(double squP) {
        this.squP = squP;
    }

    public static double inputModulus(double input, double minimumInput, double maximumInput) {
        double modulus = maximumInput - minimumInput;
        // Wrap input into the range [minimumInput, maximumInput)
        double wrapped = (input - minimumInput) % modulus;
        if (wrapped < 0) {
            wrapped += modulus;
        }
        return wrapped + minimumInput;
    }

    public double calculate(double squP, double setpoint, double measurement) {
        double error;
        if (continuous) {
            double errorBound = (maximumInput - minimumInput) / 2.0;
            error = inputModulus(setpoint - measurement, -errorBound, errorBound);
        } else {
            error = setpoint - measurement;
        }

        return squP * Math.sqrt(Math.abs(error)) * Math.signum(error);
    }

    public double calculate(double setpoint, double measurement) {
        return calculate(squP, setpoint, measurement);
    }

    public void enableContinuousInput(double minimumInput, double maximumInput) {
        continuous = true;

        this.minimumInput = minimumInput;
        this.maximumInput = maximumInput;
    }

    public static double calculateStatic(double squP, double setpoint, double measurement) {
        return squP * Math.sqrt(Math.abs(setpoint - measurement)) * Math.signum(setpoint - measurement);
    }
}