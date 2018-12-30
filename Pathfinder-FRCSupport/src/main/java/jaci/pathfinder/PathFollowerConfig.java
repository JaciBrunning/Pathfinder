package jaci.pathfinder;
/**
 * Builder-ish class that holds constants for path following.
 **/
public final class PathFollowerConfig {
    int encoderCPR;
    double wheelDiameter;
    double kP;
    double kI;
    double kD;
    double kV;
    double kA;
    double gyroP;

    public PathFollowerConfig() {
    }

    public PathFollowerConfig withEncoderCPR(int encoderCPR) {
        this.encoderCPR = encoderCPR;
        return this;
    }

    public PathFollowerConfig withWheelDiameter(double wheelDiameter) {
        this.wheelDiameter = wheelDiameter;
        return this;
    }

    public PathFollowerConfig withPIDVA(double kP,double kI, double kD, double kV, double kA) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kV = kV;
        this.kA = kA;
        return this;
    }

    public PathFollowerConfig withKP(double kP) {
        this.kP = kP;
        return this;
    }

    public PathFollowerConfig withKI(double kI) {
        this.kI = kI;
        return this;
    }

    public PathFollowerConfig withKD(double kD) {
        this.kD = kD;
        return this;
    }

    public PathFollowerConfig withKV(double kV) {
        this.kV = kV;
        return this;
    }

    public PathFollowerConfig withKA(double kA) {
        this.kA = kA;
        return this;
    }
    public PathFollowerConfig withgyroP(double gyroP) {
        this.gyroP = gyroP;
        return this;
    }
}