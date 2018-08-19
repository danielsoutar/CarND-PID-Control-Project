#include <iostream>
#include <math.h>

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

        bool record_error;
        double total_error;

        PID(double Kp, double Kd, double Ki);

        void Init(double init_CTE);

        void SetParams(int index, double value);
        void UpdateParams(int index, double value);
        void ResetParams();
        void PrintParams();

        void ResetErrors();

        double Respond(double CTE);

        void RecordTotalError();
        double GetTotalError();
};

PID::PID(double Kp, double Kd, double Ki) {
    K_coeffs[0] = Kp;
    K_coeffs[1] = Kd;
    K_coeffs[2] = Ki;
    errors[0] = errors[1] = errors[2] = 0.0;
}

void PID::Init(double init_CTE) {
    errors[0] = init_CTE;
    errors[1] = errors[2] = 0.0;
    prev_CTE = init_CTE;
    total_error = 0.0;
    record_error = false;
}

void PID::SetParams(int index, double value) {
    K_coeffs[index] = value;
}

void PID::UpdateParams(int index, double value) {
    K_coeffs[index] += value;
}

void PID::ResetParams() {
    for (int i = 0; i < NUM_PARAMS; ++i)
        K_coeffs[i] = 0.0;
}

void PID::PrintParams() {
    for (int i = 0; i < NUM_PARAMS; ++i)
        std::cout << "P" << i << ": " << K_coeffs[i] << ", ";
    std::cout << "\n";
}

void PID::ResetErrors() {
    for (int i = 0; i < NUM_PARAMS; ++i) {
        errors[i] = 0.0;
    }
}

double PID::Respond(double CTE) {
    errors[0] = CTE;
    errors[1] = CTE - prev_CTE;
    prev_CTE = CTE;
    errors[2] += CTE; 

    double alpha = -(K_coeffs[0] * errors[0]) - (K_coeffs[1] * errors[1]) - (K_coeffs[2] * errors[2]);

    if(record_error)
        total_error += pow(CTE, 2);

    return alpha;
}

void PID::RecordTotalError() {
    record_error = true;
}

double PID::GetTotalError() {
    return total_error;
}



extern "C" {
    PID* PID_new(double Kp, double Kd, double Ki){ return new PID(Kp, Kd, Ki); }
    void PID_Init(PID *pid, double init_CTE){ pid->Init(init_CTE); }
    void PID_SetParams(PID *pid, int index, double value){ pid->SetParams(index, value); }
    void PID_UpdateParams(PID *pid, int index, double value){ pid->UpdateParams(index, value); }
    void PID_ResetParams(PID *pid){ pid->ResetParams(); }
    void PID_PrintParams(PID *pid){ pid->PrintParams(); }
    void PID_ResetErrors(PID *pid){ pid->ResetErrors(); }
    double PID_Respond(PID *pid, double CTE){ return pid->Respond(CTE); }
    void PID_RecordTotalError(PID *pid){ pid->RecordTotalError(); }
    double PID_GetTotalError(PID *pid){ return pid->GetTotalError(); }
    void PID_DeleteSelf(PID *pid){ delete pid; }
}










































