class PIDController {
    private double Kp, Ki, Kd;
    private double setpoint;
    private double previousError;
    private double integral;
    private double maxOutput = 1.0;
    private double minOutput = 0.0;
    private double maxIOutput = 0.0;
    private double dt;
    private double previousDerivative = 0;
    private double derivativeFilter = 0.2;

    public PIDController(double Kp, double Ki, double Kd, double setpoint, double outputLimit, double dt) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
        this.setpoint = setpoint;
        this.dt = dt;
        setOutputLimits(0.0, outputLimit);
    }

    public void setOutputLimits(double minimum, double maximum) {
        if (maximum < minimum) return;
        maxOutput = maximum;
        minOutput = minimum;
        if (maxIOutput == 0 || maxIOutput > (maximum - minimum)) {
            setMaxIOutput(maximum - minimum);
        }
    }

    public void setMaxIOutput(double maximum) {
        maxIOutput = maximum;
    }

    public double calculate(double processVariable) {
        double error = setpoint - processVariable;

        // Proportional
        double Pout = Kp * error;

        // Integral
        if (Ki != 0) {
            integral += Ki * error * dt;
            integral = Math.max(-maxIOutput, Math.min(maxIOutput, integral));
        }

        // Filtered derivative
        double derivative = (error - previousError) / dt;
        double Dout = Kd * ((1 - derivativeFilter) * derivative + derivativeFilter * previousDerivative);

        // Calculate output
        double output = Pout + integral + Dout;
        output = Math.max(minOutput, Math.min(maxOutput, output));

        // Save state
        previousError = error;
        previousDerivative = derivative;

        return output;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void resetIntegral() {
        this.integral = 0;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setGains(double Kp, double Ki, double Kd) {
        this.Kp = Kp;
        this.Ki = Ki;
        this.Kd = Kd;
    }
}