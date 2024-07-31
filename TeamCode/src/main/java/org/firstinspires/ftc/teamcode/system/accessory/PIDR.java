package org.firstinspires.ftc.teamcode.system.accessory;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.system.paths.P2P.LowPassFilter;

public class PIDR
{
    // This was never tested lol, it just exists
    ElapsedTime timer = new ElapsedTime();
    double Kp;
    double Ki;
    double Kd;
    double integralSumLimit; // max integral value, should have its own constructor variable to change
    double lastError;
    double integralSum;
    double error;
    double errorDerivative;
    double derivative;
    double output;
    double Kf;
    double integralWrap;

    // Low pass filter
    double previousEstimate;
    double currentEstimate;
    double a;

    // Integral wind up
    double reference;
    double lastReference;



    public PIDR(double Kp, double Ki, double Kd, double integralSumLimit, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.integralSumLimit = integralSumLimit;
        this.Kf = Kf;
        previousEstimate = 0;
        currentEstimate = 0;
    }
    public PIDR(double Kp, double Ki, double Kd, double Kf){
        this.Kp = Kp;
        this.Kd = Kd;
        this.Ki = Ki;
        this.integralSumLimit = 0.25 / Ki; // this make the sum limit be 0.25 = ki * sumlimit
        this.Kf = Kf;
        previousEstimate = 0;
        currentEstimate = 0;
    }

    public double update(double target, double state, double maxOutput) { // parameter of the method is the target,
        // PID logic and then return the output
        error = target-state;
        derivative = (error-lastError)/timer.seconds();
        integralSum = integralSum + (error * timer.seconds());

        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * target);

        if (output > maxOutput){ // basically cuts the PID so the motor can run at the max speed
            output = maxOutput;
        } else if (output < -maxOutput){
            output = -maxOutput; // accounts for negative as well
        }

        lastError = error;
        timer.reset(); // i didn't do this it will mess with the PIDs

        return output;
    }
    double filterCoeff = 0.8;
    LowPassFilter derivativeFilter = new LowPassFilter(filterCoeff, 0);
    /** Return the output with a low pass filter and an integral wind up */
    public double betterUpdate(double target, double state, double maxOutput) { // parameter of the method is the target,
        // PID logic and then return the output with low pass filter + integral wind up
        error = target-state;
        errorDerivative = error-lastError;
        derivative = (error-lastError) / timer.seconds();
        reference = Math.signum(error);

        // Low pass filter logic
        //currentEstimate = (a * previousEstimate) + (1 - a) * errorDerivative;
        //derivative = currentEstimate / timer.seconds();
        derivative = derivativeFilter.getValue(derivative);
        //previousEstimate = currentEstimate;

        integralSum += error * timer.seconds();
        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }

        if (reference != lastReference) // idea is that if the sign of the error changes we reset the integralSum
        {
            integralSum = 0;
        }

        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative) + (Kf * target);

        if (output > maxOutput){ // basically cuts the PID so the motor can run at the max speed
            output = maxOutput;
        } else if (output < -maxOutput){
            output = -maxOutput; // accounts for negative as well
        }
        lastReference = reference;
        lastError = error;
        timer.reset(); // i didn't do this it will mess with the PIDs

        return output;
    }

    public double updateWithError(double errorHeading, double maxOutput) { // parameter of the method is the target,
        // PID logic and then return the output
        error = errorHeading;
        derivative = (error-lastError)/timer.seconds();
        integralSum = integralSum + (error * timer.seconds());
        // set a limit on our integral sum
        if (integralSum > integralSumLimit) {
            integralSum = integralSumLimit;
        }
        if (integralSum < -integralSumLimit) {
            integralSum = -integralSumLimit;
        }
        output = (Kp * error) + (Ki * integralSum) + (Kd * derivative);

        if (output > maxOutput){ // basically cuts the PID so the motor can run at the max speed
            output = maxOutput;
        } else if (output < -maxOutput){
            output = -maxOutput; // accounts for negative as well
        }

        lastError = error;
        timer.reset(); // i didn't do this it will mess with the PIDs

        return output;
    }


    public double returnError(){
        return error;
    }
    public double returnOutput () {return output;}

    public double returnIntegralSum() {
        return integralSum;
    }
    public double returnDerivative()
    {
        return derivative;
    }

}


