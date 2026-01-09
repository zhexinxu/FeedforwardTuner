#include <Arduino.h>
#include "FeedforwardTuner.h"

FeedforwardTuner::FeedforwardTuner(
    double cvMin, double cvMax,
    int numSamples,
    unsigned long settleTimeMs,
    const double &pvRef,
    double &cvRef
)
: _cvMin(cvMin), _cvMax(cvMax),
  _numSamples(numSamples), _settleTime(settleTimeMs),
  _pvRef(pvRef), _cvRef(cvRef)
{
    _numSamples = max(numSamples, 2);
    _cv = new double[_numSamples];
    _pv = new double[_numSamples];

    _state = SET_CV;
    _index = 0;
    _stateStartTime = millis();
    _b0 = 0;
    _b1 = 0;
}

bool FeedforwardTuner::IsFinished() const {
    return _state == DONE;
}

void FeedforwardTuner::RunTime() {
    if (_state == DONE) return;

    unsigned long now = millis();

    switch (_state) {

        case SET_CV:
            _cvRef = _cvMin +
                (_cvMax - _cvMin) * ((double)_index / (_numSamples - 1));
            _stateStartTime = now;
            _state = WAIT_SETTLE;
            break;

        case WAIT_SETTLE:
            if (now - _stateStartTime >= _settleTime)
                _state = RECORD_POINT;
            break;

        case RECORD_POINT:
            _cv[_index] = _cvRef;
            _pv[_index] = _pvRef;

            _index++;

            if (_index >= _numSamples) {
                ComputeOLS();
                _state = DONE;
            } else {
                _state = SET_CV;
            }
            break;

        default:
            break;
    }
}

void FeedforwardTuner::Reset() {
    _state = SET_CV;
    _index = 0;
    _stateStartTime = millis();
}

void FeedforwardTuner::ComputeOLS() {
    double sumX=0, sumY=0, sumXY=0, sumXX=0;

    for (int i=0; i<_numSamples; i++) 
    {
        double x = _pv[i];
        double y = _cv[i];
        sumX  += x;
        sumY  += y;
        sumXY += x*y;
        sumXX += x*x;
    }

    double meanX = sumX / _numSamples;
    double meanY = sumY / _numSamples;

    double num = sumXY - _numSamples * meanX * meanY;
    double den = sumXX - _numSamples * meanX * meanX;

    if (fabs(den) < 1e-9) 
    {
        _b0 = 0.0;
        _b1 = 0.0;
    }
    else
    {
        _b1 = num / den;
        _b0 = meanY - _b1 * meanX;
    }
}

FeedforwardTuner::~FeedforwardTuner()
{
    delete[] _cv;
    delete[] _pv;
}

