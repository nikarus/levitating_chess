#include "regulator_settings.h"

float deadtime_voltage(float target)
{
    float argument = target / DEADTIME_CURRENT_SCALE;
    float magnitude = argument < 0.0f ? -argument : argument;
    float value;
    if (magnitude >= DEADTIME_DOMAIN) {
        value = 1.0f;
    } else {
        float position = magnitude * DEADTIME_INDEX_SCALE;
        unsigned index = (unsigned)position;
        if (index >= DEADTIME_INTERVALS) index = DEADTIME_INTERVALS - 1u;
        value = deadtime_table[index] + (position - (float)index) *
            (deadtime_table[index + 1u] - deadtime_table[index]);
    }
    return DEADTIME_VOLTAGE * (argument < 0.0f ? -value : value);
}

static unsigned nearest_even(float value)
{
    unsigned integer = (unsigned)value;
    float remainder = value - (float)integer;
    return integer + (remainder > 0.5f || (remainder == 0.5f && (integer & 1u)));
}

static unsigned regulator_step(float error, float integral, float target, float target_rate, float *result)
{
    float feedforward = COIL_PATH_RESISTANCE * target + COIL_INDUCTANCE * target_rate;
    feedforward += deadtime_voltage(target);
    float requested = COIL_INDUCTANCE * LOOP_ANGULAR_FREQUENCY * error + integral + feedforward;
    float voltage = requested < -VOLTAGE_LIMIT ? -VOLTAGE_LIMIT : requested;
    voltage = voltage > VOLTAGE_LIMIT ? VOLTAGE_LIMIT : voltage;
    if (voltage == requested || error * requested < 0.0f) {
        integral += COIL_PATH_RESISTANCE * LOOP_ANGULAR_FREQUENCY * INTEGRAL_RATIO * error / REGULATION_RATE;
    }
    float positive = 0.5f * (1.0f + voltage / BUS_VOLTAGE);
    float negative = 0.5f * (1.0f - voltage / BUS_VOLTAGE);
    unsigned positive_compare = nearest_even(positive * PWM_HALF_COUNTS);
    unsigned negative_compare = nearest_even(negative * PWM_HALF_COUNTS);
    result[0] = ((float)positive_compare - (float)negative_compare) * BUS_VOLTAGE / PWM_HALF_COUNTS;
    result[1] = integral;
    result[2] = requested;
    return positive_compare | (negative_compare << PWM_COMPARE_BITS);
}

void regulate_current(float error, float integral, float target, float target_rate, float *result)
{
    regulator_step(error, integral, target, target_rate, result);
}

typedef struct {
    unsigned long long frame_index;
    unsigned long long epoch_tick;
    unsigned long long last_service_tick;
    unsigned first_code;
    unsigned fault;
    unsigned calibrating;
    float zero_offset[CONTROL_CHANNELS];
    float integral[CONTROL_CHANNELS];
    float reference[CONTROL_CHANNELS][REFERENCE_STATES];
} CurrentRuntime;

unsigned current_runtime_size(void)
{
    return sizeof(CurrentRuntime);
}

unsigned current_runtime_fault(const CurrentRuntime *runtime)
{
    return runtime->fault;
}

static int runtime_fault(CurrentRuntime *runtime, unsigned fault)
{
    runtime->fault = fault;
    return -1;
}

void current_runtime_initialize(CurrentRuntime *runtime, unsigned long long epoch_tick, unsigned configuration_verified)
{
    runtime->frame_index = 0;
    runtime->epoch_tick = epoch_tick;
    runtime->last_service_tick = epoch_tick;
    runtime->first_code = 0;
    runtime->calibrating = 0;
    runtime->fault = configuration_verified ? 0 : 1;
    for (unsigned channel = 0; channel < CONTROL_CHANNELS; ++channel) {
        runtime->integral[channel] = 0.0f;
        runtime->zero_offset[channel] = 0.0f;
        for (unsigned state = 0; state < REFERENCE_STATES; ++state) runtime->reference[channel][state] = 0.0f;
    }
}

int current_runtime_begin_zero_calibration(CurrentRuntime *runtime)
{
    if (runtime->fault) return -1;
    if (runtime->frame_index) return runtime_fault(runtime, 12);
    for (unsigned channel = 0; channel < CONTROL_CHANNELS; ++channel)
        if (runtime->reference[channel][REFERENCE_STATES-1u] != 0.0f) return runtime_fault(runtime, 12);
    runtime->calibrating = CONTROL_CHANNELS;
    return 0;
}

unsigned current_runtime_calibration_remaining(const CurrentRuntime *runtime)
{
    return runtime->calibrating;
}

static unsigned long long runtime_deadline(const CurrentRuntime *runtime)
{
    return runtime->epoch_tick + runtime->frame_index / SAMPLES_PER_PAIR * CARRIER_TICKS +
        channel_latch_ticks[runtime->frame_index / SAMPLES_PER_PAIR % CONTROL_CHANNELS] - COMPUTE_TICKS;
}

int current_runtime_check_deadline(CurrentRuntime *runtime, unsigned long long now_tick)
{
    if (runtime->fault) return -1;
    if (now_tick > runtime_deadline(runtime)) return runtime_fault(runtime, 2);
    return 0;
}

int current_runtime_command(CurrentRuntime *runtime, unsigned channel, float command)
{
    if (runtime->fault) return -1;
    if (channel >= CONTROL_CHANNELS || !__builtin_isfinite(command) || command < -COMMAND_CURRENT_LIMIT || command > COMMAND_CURRENT_LIMIT)
        return runtime_fault(runtime, 3);
    if (runtime->calibrating && command != 0.0f) return runtime_fault(runtime, 12);
    runtime->reference[channel][REFERENCE_STATES-1u] = command;
    return 0;
}

int current_runtime_frame(CurrentRuntime *runtime, unsigned long long frame_index, unsigned first_word, unsigned second_word,
    unsigned long long available_tick, unsigned transfer_error, unsigned *channel_output, unsigned *pwm_output,
    unsigned long long *latch_tick, float *measurement_output, float *voltage_output)
{
    if (runtime->fault) return -1;
    if (transfer_error) return runtime_fault(runtime, 4);
    if (frame_index != runtime->frame_index) return runtime_fault(runtime, 5);
    if (available_tick < runtime->last_service_tick) return runtime_fault(runtime, 6);
    if (available_tick < runtime->epoch_tick + frame_index * SAMPLE_TICKS + FIFO_READY_TICKS) return runtime_fault(runtime, 7);
    if (current_runtime_check_deadline(runtime, available_tick)) return -1;
    unsigned partial_bits = FRAME_BITS - FIFO_WORD_BITS;
    if (second_word >= (1u << partial_bits)) return runtime_fault(runtime, 8);
    unsigned long long frame = ((unsigned long long)first_word << partial_bits) | second_word;
    unsigned trailing_bits = FRAME_BITS - ADC_COMMAND_BITS - ADC_BITS - ADC_TAG_BITS;
    unsigned channel = (unsigned)(frame_index / SAMPLES_PER_PAIR % CONTROL_CHANNELS);
    if ((frame >> (FRAME_BITS - ADC_COMMAND_BITS)) || (frame & ((1u << trailing_bits)-1u)) ||
        ((frame >> trailing_bits) & ((1u << ADC_TAG_BITS)-1u)) != channel) return runtime_fault(runtime, 9);
    unsigned code = (unsigned)(frame >> (trailing_bits + ADC_TAG_BITS)) & ((1u << ADC_BITS)-1u);
    if (code == 0 || code == ((1u << ADC_BITS)-1u)) return runtime_fault(runtime, 11);
    runtime->last_service_tick = available_tick;
    runtime->frame_index++;
    if (frame_index % SAMPLES_PER_PAIR == 0) {
        runtime->first_code = code;
        return 0;
    }
    float measured = (((float)runtime->first_code + (float)code) / SAMPLES_PER_PAIR + 0.5f) * ADC_CURRENT_SCALE - ADC_CURRENT_BIAS;
    if (runtime->calibrating) {
        runtime->zero_offset[channel] = measured;
        runtime->calibrating--;
    }
    measured -= runtime->zero_offset[channel];
    float targets[3], advanced[REFERENCE_STATES], result[3];
    channel_reference_step(channel, runtime->reference[channel], targets, advanced);
    unsigned compares = regulator_step(targets[2]-measured, runtime->integral[channel], targets[0], targets[1], result);
    if (!__builtin_isfinite(result[0]) || !__builtin_isfinite(result[1]) || !__builtin_isfinite(result[2]))
        return runtime_fault(runtime, 10);
    for (unsigned state = 0; state < REFERENCE_STATES; ++state) runtime->reference[channel][state] = advanced[state];
    runtime->integral[channel] = result[1];
    *channel_output = channel;
    *pwm_output = compares;
    *latch_tick = runtime->epoch_tick + frame_index / SAMPLES_PER_PAIR * CARRIER_TICKS + channel_latch_ticks[channel];
    *measurement_output = measured;
    *voltage_output = result[0];
    return 1;
}

int current_runtime_pair(CurrentRuntime *runtime, unsigned long long first_frame, const unsigned *words,
    unsigned long long available_tick, unsigned transfer_error, unsigned *channel_output, unsigned *pwm_output,
    unsigned long long *latch_tick, float *measurement_output, float *voltage_output)
{
    if (first_frame % SAMPLES_PER_PAIR) return runtime_fault(runtime, 5);
    int first = current_runtime_frame(runtime, first_frame, words[0], words[1], available_tick, transfer_error,
        channel_output, pwm_output, latch_tick, measurement_output, voltage_output);
    if (first != 0) return -1;
    return current_runtime_frame(runtime, first_frame+1u, words[2], words[3], available_tick, transfer_error,
        channel_output, pwm_output, latch_tick, measurement_output, voltage_output);
}
