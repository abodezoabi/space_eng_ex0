public class Engine {
    private double thrust; // Newtons
    private double burnRate; // Fuel consumption per second (kg/s)
    private boolean active; // Engine state

    public Engine(double thrust, double burnRate) {
        this.thrust = thrust;
        this.burnRate = burnRate;
        this.active = false;
    }

    public double getThrust() {
        return active ? thrust : 0;
    }

    public double getBurnRate() {
        return active ? burnRate : 0;
    }

    public void activate() {
        this.active = true;
    }

    public void deactivate() {
        this.active = false;
    }
}
