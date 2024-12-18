package frc.robot.lib;

import java.util.ArrayList;

public class Filter {
    double[] x = {0,0,0,0,0};
    double[] v = {0,0,0,0,0};

    double tick;

    public Filter() {

    }

    /**
     * @param xInput Units
     * @param vInput Units per second
     * @return the filtered value
     */
    public double calculate(double xInput, double vInput) {
        tick++;
        for (int i = 4; i > 0; i--) {
            x[i] = x[i-1];
            v[i] = v[i-1];
        }
        x[0] = xInput;
        v[0] = vInput;


        if (tick < 5) {
            return xInput;
        }

        double[] f = {0,0,0,0,0};
        f[0] = x[0];
        for (int i = 1; i <= 4; i++) {
            f[i] = x[i];
            for (int j = 1; j <= i; j++) {
                f[i] = f[i] + (v[j] * .020);
            }
        }

        double sum = 0;
        for (int i = 0; i < 5; i++) {
            sum += f[i];
        }

        return sum/5;
    }
}
