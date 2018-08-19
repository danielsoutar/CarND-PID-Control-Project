#ifndef PID_H
#define PID_H

class PID {
    public:

        const static int NUM_PARAMS = 3;

        /*
        * Errors
        */
        double errors[NUM_PARAMS]; // p_error, d_error, i_error, in that order

        /*
        * Coefficients
        */ 
        double K_coeffs[NUM_PARAMS];  // Kp, Kd, Ki, in that order

        double prev_CTE;
        double prev_angle;

        bool record_error;
        double total_error;

        PID();

        PID(double Kp, double Kd, double Ki);

        /*
        * Destructor.
        */
        virtual ~PID();

        void Init(double init_CTE);

        void SetParams(int index, double value);
        void UpdateParams(int index, double value);
        void ResetParams();
        void PrintParams();

        void ResetErrors();

        double Respond(double CTE);
        double RespondThrottle(double angle, double max_velocity);

        void RecordTotalError();
        double GetTotalError();
};

#endif /* PID_H */
