#ifndef _sweeper_h
#define _sweeper_h

struct SweeperState {

    unsigned int startHz;
    unsigned int endHz;
    unsigned int stepHz;

    enum State {
        PAUSED,
        READY,
        SENDING,
        DONE
    } state;

    unsigned int freqHz;

    // Here is where the power observation are stored
    static const unsigned int maxSampleCount = 100;
    unsigned int sampleCount = 0;
    unsigned int sample[maxSampleCount];
};

class SweeperContext {
public:

    virtual void setFreq(unsigned int freqHz) = 0;
    virtual unsigned int getRMS() const = 0;
};

void sweeper_tick(SweeperState* state, SweeperContext* context);

#endif

