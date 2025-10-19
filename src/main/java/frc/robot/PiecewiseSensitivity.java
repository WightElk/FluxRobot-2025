package frc.robot;

/**
 *
 */
class PiecewiseSensitivity {
    double xStart;
    double xMiddle;
    double yStart;
    double yMiddle;
    double yMax;
    double slope1;
    double slope2;

    public PiecewiseSensitivity(double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
        set(xStart, xMiddle, yStart, yMiddle, yMax);
    }

    public void set(double xStart, double xMiddle, double yStart, double yMiddle, double yMax) {
        this.xStart = xStart;
        this.xMiddle = xMiddle;
        this.yStart = yStart;
        this.yMiddle = yMiddle;
        this.yMax = yMax;
        slope1 = xMiddle < xStart ? (yMiddle - yStart) / (xMiddle - xStart) : 0.0;
        slope2 = xMiddle < 1.0 ? (yMax - yMiddle) / (1.0 - xMiddle) : 0.0;
    }

    double transfer(double x) {
        double xabs = Math.abs(x);
        if (xabs < xStart) {
            return 0;
        }
        else if (xabs < xMiddle) {
            xabs = yStart + slope1 * (xabs - xStart);
        }
        else if (xabs < 1.0) {
            xabs = yMiddle + slope2 * (xabs - xMiddle);
        }
        else {
            xabs = yMax;
        }
        return x >= 0 ? xabs : -xabs;
    }
}
