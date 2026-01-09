#ifndef FEEDFORWARDTUNER_H
#define FEEDFORWARDTUNER_H

// #include <Arduino.h>

class FeedforwardTuner 
{
public:
    FeedforwardTuner(
        double cvMin, double cvMax,      // sweep range
        int numSamples,                // number of (CV,PV) samples
        unsigned long settleTimeMs,    // wait time for PV settling
        const double &pvRef,            // reference to PV variable
        double &cvRef            // reference to CV
    );

    ~FeedforwardTuner();
    FeedforwardTuner(const FeedforwardTuner&) = delete;
    FeedforwardTuner& operator=(const FeedforwardTuner&) = delete;

    // Call from loop()
    void RunTime();

    void Reset();

    bool IsFinished() const;

    double GetB0() const { return _b0; }

    double GetB1() const { return _b1; }

private:
    enum State {
        SET_CV,
        WAIT_SETTLE,
        RECORD_POINT,
        DONE
    };

    void ComputeOLS();

    double *_cv;
    double *_pv;
    int _numSamples;

    double _cvMin, _cvMax;
    unsigned long _settleTime;
    const double &_pvRef;   // reference to PV variable
    double &_cvRef;

    State _state;
    int _index;

    unsigned long _stateStartTime;

    double _b0, _b1;
};

#endif
