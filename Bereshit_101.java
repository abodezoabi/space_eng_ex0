import java.io.FileWriter;
import java.io.IOException;

public class Bereshit_101 {
    public static final double WEIGHT_EMP = 165;
    public static final double WEIGHT_FUEL = 420;
    public static final double WEIGHT_FULL = WEIGHT_EMP + WEIGHT_FUEL;
    public static final double MAIN_ENG_F = 430;
    public static final double SECOND_ENG_F = 25;
    public static final double MAIN_BURN = 0.15;
    public static final double SECOND_BURN = 0.009;
    public static final double ALL_BURN = MAIN_BURN + 8 * SECOND_BURN;
    public static final double DT = 1.0;
    public static final double MOON_GRAVITY = 1.62;

    public static double accMax(double weight) {
        return acc(weight, true, 8);
    }

    public static double acc(double weight, boolean main, int seconds) {
        double thrust = 0;
        if (main) thrust += MAIN_ENG_F;
        thrust += seconds * SECOND_ENG_F;
        return thrust / weight;
    }

    public static void main(String[] args) {
        System.out.println("Simulating Bereshit 101 Lunar Landing...");
        System.out.println("----------------------------------------------------------");
        System.out.println(" Time  |   VS   |   HS   |  Dist  |  Alt   |  Ang  | Weight |  Acc  | NN | Phase");
        System.out.println("----------------------------------------------------------");

        try (FileWriter writer = new FileWriter("bereshit_data.csv")) {
            writer.write("time,vs,hs,dist,alt,ang,weight,acc,nn,phase\n");

            // Initial conditions - POSITIVE VS for descent
            double vs = 47.3;  // POSITIVE for descent
            double hs = 1698;
            double dist = 181000;
            double ang = 58.3;
            double alt = 30000;  
            double time = 0;
            double acc = 0;
            double fuel = 221;
            double weight = WEIGHT_EMP + fuel;
            double NN = 0.7;

            // PID Controller - tuned for descent
            PIDController pid = new PIDController(
                3.0, 0.5, 1.5, 0, 1.0, DT
            );
            pid.setOutputLimits(0.7, 1.0);

            // Phase 1: Adjust angle to 0 first (max 100 iterations)
            int angleAdjustCount = 0;
            while (ang > 0 && angleAdjustCount++ < 100) {
                ang = Math.max(0, ang - 1.5);  // Faster angle reduction
                
                // Minimal braking during angle adjustment
                NN = 0.7;
                acc = NN * accMax(weight);

                // Physics calculations
                double ang_rad = Math.toRadians(ang);
                double h_acc = Math.sin(ang_rad) * acc;
                double v_acc = Math.cos(ang_rad) * acc - MOON_GRAVITY;

                // Update state
                time += DT;
                if (fuel > 0) {
                    double dw = DT * ALL_BURN * NN;
                    fuel -= dw;
                    weight = WEIGHT_EMP + fuel;
                }
                hs = Math.max(0, hs - h_acc * DT);
                vs -= v_acc * DT;
                dist -= hs * DT;
                alt -= vs * DT;

                // Safety check - prevent negative altitude
                if (alt <= 0) {
                    alt = 0;
                    break;
                }

                // Log data
                if (time % 10 == 0 || alt < 100) {
                    logData(writer, time, vs, hs, dist, alt, ang, weight, acc, NN, "AngleAdj");
                    printStatus(time, vs, hs, dist, alt, ang, weight, acc, NN, "AngleAdj");
                }
            }

            // Phase 2: Main braking after angle is 0 (max 1000 iterations)
            int brakingCount = 0;
            while (alt > 100 && brakingCount++ < 1000) {
                pid.setSetpoint(10);  // Target VS during braking

                // Calculate braking with safety checks
                double maxPossibleAcc = accMax(weight);
                double gravityComp = MOON_GRAVITY / maxPossibleAcc;
                double pidOutput = pid.calculate(Math.abs(vs));  // Use absolute value
                NN = Math.max(0.7, Math.min(1.0, pidOutput + gravityComp * 0.8));
                acc = NN * maxPossibleAcc;

                // Reduce horizontal speed
                hs *= 0.85;
                hs = Math.max(0, hs);

                // Vertical braking only (angle is 0)
                double v_acc = acc - MOON_GRAVITY;

                // Update state with safety checks
                time += DT;
                if (fuel > 0) {
                    double dw = DT * ALL_BURN * NN;
                    fuel -= dw;
                    weight = WEIGHT_EMP + fuel;
                }
                vs -= v_acc * DT;
                vs = Math.max(0, vs);  // Prevent negative VS
                dist -= hs * DT;
                alt -= vs * DT;
                alt = Math.max(0, alt);  // Prevent negative altitude
                if(alt<=0){
                    NN = 0;
                    break;}

                // Log data
                if (time % 10 == 0 || alt < 100) {
                    logData(writer, time, vs, hs, dist, alt, ang, weight, acc, NN, "Braking");
                    printStatus(time, vs, hs, dist, alt, ang, weight, acc, NN, "Braking");
                }

                // Emergency exit if not making progress
                if (vs > 1000 || alt > 1e6) {
                    System.out.println("EMERGENCY: Velocity or altitude too high - aborting");
                    break;
                }
                if (alt<=10){
                     NN = 0;
                     break;}
            }

            // Phase 3: Final landing (max 200 iterations)
            int landingCount = 0;
            while (alt > 10 && landingCount++ < 200) {
                
                pid.setSetpoint(2.5);  // Soft landing target

                // Precise braking with safety checks
                double maxPossibleAcc = accMax(weight);
                double gravityComp = MOON_GRAVITY / maxPossibleAcc;
                double pidOutput = pid.calculate(Math.abs(vs));
                NN = Math.max(0.7, Math.min(1.0, pidOutput + gravityComp));
                acc = NN * maxPossibleAcc;

                // Eliminate horizontal speed
                hs *= 0.8;
                hs = Math.max(0, hs);

                // Vertical braking only
                double v_acc = acc - MOON_GRAVITY;

                // Update state with safety checks
                time += DT;
                if (fuel > 0) {
                    double dw = DT * ALL_BURN * NN;
                    fuel -= dw;
                    weight = WEIGHT_EMP + fuel;
                }
                vs -= v_acc * DT;
                vs = Math.max(0, vs);  // Prevent negative VS
                dist -= hs * DT;
                alt -= vs * DT;
                alt = Math.max(0, alt);  // Prevent negative altitude

                // Log data
                if (time % 1 < DT || alt < 10) {
                    logData(writer, time, vs, hs, dist, alt, ang, weight, acc, NN, "Final");
                    printStatus(time, vs, hs, dist, alt, ang, weight, acc, NN, "Final");
                }
                if(alt<=10){
                    NN = 0;
                    break;}
            }

            // Landing results
            printLandingResults(time, vs, hs, fuel);

        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    private static void logData(FileWriter writer, double time, double vs, double hs,
                              double dist, double alt, double ang, double weight,
                              double acc, double NN, String phaseName) throws IOException {
        String data = String.format("%.1f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%s",
                time, vs, hs, dist, alt, ang, weight, acc, NN, phaseName);
        writer.write(data + '\n');
    }

    private static void printStatus(double time, double vs, double hs,
                                  double dist, double alt, double ang, double weight,
                                  double acc, double NN, String phaseName) {
        System.out.printf("%5.1f | %6.2f | %6.2f | %6.0f | %6.0f | %5.1f | %6.1f | %5.2f | %.2f | %s\n",
                time, vs, hs, dist, alt, ang, weight, acc, NN, phaseName);
    }

    private static void printLandingResults(double time, double vs, double hs, double fuel) {
        System.out.println("\n--- LANDING RESULTS ---");
        System.out.printf("Touchdown at: %.1f seconds\n", time);
        System.out.printf("Final VS: %.2f m/s, HS: %.2f m/s\n", vs, hs);
        System.out.printf("Remaining fuel: %.1f kg\n", fuel);
        
        if (Math.abs(vs) < 3 && hs < 1) {
            System.out.println("SUCCESS! Safe landing achieved!");
        } else {
            System.out.println("FAILURE! Crash landing!");
        }
    }
}
