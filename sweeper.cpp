
#include <cstdio>
#include "sweeper.h"

void sweeper_tick(SweeperState* state, SweeperContext* context) {

    if (state->state == SweeperState::State::PAUSED) {
        return;
    }
    else if (state->state == SweeperState::State::READY) {
        state->sampleCount = 0;
        state->freqHz = state->startHz;
        context->setFreq(state->freqHz);
        state->state = SweeperState::State::SENDING;
    }
    else if (state->state == SweeperState::State::SENDING) {
        // Take a sample
        state->sample[state->sampleCount] = context->getRMS();
        state->sampleCount++;
        // LSB sweep!
        state->freqHz += state->stepHz;
        if (state->sampleCount == state->maxSampleCount ||
            state->freqHz > state->endHz) {
            state->state = SweeperState::State::DONE;
            printf("Sweep done\n");
        }
        else {
            context->setFreq(state->freqHz);
            state->state = SweeperState::State::SENDING;
        }
    }

}