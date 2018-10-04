#include <asrc/asrc.h>

#include <samplerate.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>
using namespace std;

namespace
{
    using clock = chrono::high_resolution_clock;

    static unsigned nextPowerOf2(unsigned v)
    {
        v--;
        v |= v >> 1;
        v |= v >> 2;
        v |= v >> 4;
        v |= v >> 8;
        v |= v >> 16;
        v++;
        return v;
    }

    struct fifo {
        fifo(unsigned size)
            : buffer(nextPowerOf2(size))
            , mask(unsigned(buffer.size() - 1))
            , write_pos(0)
            , read_pos(0)
        {
        }
        void push(const float* input, unsigned num_frames)
        {
            unsigned offset = 0;
            while (num_frames > 0) {
                unsigned wr_pos     = write_pos & mask;
                unsigned frames_now = std::min(num_frames, unsigned(buffer.size()) - wr_pos);
                copy_n(input + offset, frames_now, buffer.data() + wr_pos);
                write_pos += frames_now;
                offset += frames_now;
                num_frames -= frames_now;
            }
        }
        void pop(float* output, unsigned num_frames)
        {
            if (count() < num_frames) {
                fill_n(output, num_frames, 0.f);
            } else {
                unsigned offset = 0;
                while (num_frames > 0) {
                    unsigned rd_pos     = read_pos & mask;
                    unsigned frames_now = min(num_frames, unsigned(buffer.size()) - rd_pos);
                    copy_n(buffer.data() + rd_pos, frames_now, output + offset);
                    read_pos += frames_now;
                    offset += frames_now;
                    num_frames -= frames_now;
                }
            }
        }

    private:
        unsigned count() const { return write_pos - read_pos; }
        vector<float> buffer;
        const unsigned mask;
        atomic<unsigned> write_pos;
        atomic<unsigned> read_pos;
    };

    struct asrc {
        atomic<double> input_rate;
        atomic<double> output_rate;
        atomic<double> current_ratio;
        atomic<double> adaption_speed;
        clock::duration input_timestamp;
        clock::duration output_timestamp;
        const int in_buf_size;
        const int out_buf_size;

        vector<unique_ptr<fifo>> fifos;
        vector<float> src_buffer;

        SRC_STATE* src_handle{nullptr};

        std::thread update_thread;
        atomic<bool> stop_update_thread{false};

        asrc(int quality, int channels, int in_buf_size, int out_buf_size, double initial_ratio)
            : in_buf_size(in_buf_size)
            , out_buf_size(out_buf_size)
            , current_ratio(initial_ratio)
            , src_buffer(size_t(2 * out_buf_size * initial_ratio))
        {
            int error  = 0;
            src_handle = src_new(quality, channels, &error);
            if (error != 0 || src_handle == NULL)
                throw runtime_error("failed to initialize libsamplerate, code: " +
                                    to_string(error));
            // Create FIFO buffers with at least 4 * out_buf_size number of samples
            for (int i = 0; i < channels; ++i) {
                auto f = make_unique<fifo>(4 * out_buf_size);
                fifos.push_back(move(f));
            }

            std::thread t([this] {
                while (!stop_update_thread) {
                    this_thread::sleep_for(chrono::milliseconds(20));
                    if (input_rate != 0.0 && output_rate != 0.0) {
                        auto factor = output_rate / input_rate;
                        current_ratio =
                            (1.0 - adaption_speed) * current_ratio + adaption_speed * factor;
                    }
                }
            });
            update_thread.swap(t);
        }
        ~asrc()
        {
            if (update_thread.joinable()) {
                stop_update_thread = true;
                update_thread.join();
            }
            src_delete(src_handle);
        }

        void set_adaption_speed(double speed) { adaption_speed = speed; }

        void process_input(const float** inputs)
        {
            //
            auto t_now      = clock::now();
            auto t_diff     = t_now - clock::time_point(input_timestamp);
            input_timestamp = t_now.time_since_epoch();
            input_rate =
                double(in_buf_size) / chrono::duration_cast<chrono::microseconds>(t_diff).count();

            for (size_t ch_idx = 0; ch_idx < fifos.size(); ++ch_idx) {
                SRC_DATA data;
                data.data_in       = inputs[ch_idx];
                data.data_out      = src_buffer.data();
                data.input_frames  = in_buf_size;
                data.output_frames = long(src_buffer.size());
                data.src_ratio     = current_ratio;
                src_process(src_handle, &data);
                fifos[ch_idx]->push(src_buffer.data(), data.output_frames_gen);
            }
        }

        void process_output(float** outputs)
        {
            //
            auto t_now       = clock::now();
            auto t_diff      = t_now - clock::time_point(output_timestamp);
            output_timestamp = t_now.time_since_epoch();
            output_rate =
                double(out_buf_size) / chrono::duration_cast<chrono::microseconds>(t_diff).count();

            for (size_t ch_idx = 0; ch_idx < fifos.size(); ++ch_idx) {
                fifos[ch_idx]->pop(outputs[ch_idx], out_buf_size);
            }
        }
    };
}

ASRC_RESULT asrc_create(ASRC_HANDLE* handle,
                        ASRC_QUALITY quality,
                        int channels,
                        int in_buffer_size,
                        int out_buffer_size,
                        double init_ratio)
{
    if (handle == nullptr)
        return ASRC_INVALID_ARGUMENT;

    auto p  = new asrc(quality, channels, in_buffer_size, out_buffer_size, init_ratio);
    *handle = (void*)p;
    return ASRC_SUCCESS;
}

ASRC_RESULT asrc_adaption_speed(ASRC_HANDLE handle, double speed)
{
    if (handle == nullptr)
        return ASRC_INVALID_ARGUMENT;
    if (speed < 0.001 || speed > 1.0)
        return ASRC_INVALID_ARGUMENT;
    auto p = reinterpret_cast<asrc*>(handle);
    p->set_adaption_speed(speed);
    return ASRC_SUCCESS;
}

ASRC_RESULT asrc_process_input(ASRC_HANDLE handle, const float** inputs)
{
    if (handle == nullptr || inputs == nullptr)
        return ASRC_INVALID_ARGUMENT;
    auto p = reinterpret_cast<asrc*>(handle);
    p->process_input(inputs);
    return ASRC_SUCCESS;
}

ASRC_RESULT asrc_process_output(ASRC_HANDLE handle, float** outputs)
{
    if (handle == nullptr || outputs == nullptr)
        return ASRC_INVALID_ARGUMENT;
    auto p = reinterpret_cast<asrc*>(handle);
    p->process_output(outputs);
    return ASRC_SUCCESS;
}

void asrc_destroy(ASRC_HANDLE handle)
{
    if (handle != nullptr) {
        auto p = reinterpret_cast<asrc*>(handle);
        delete p;
    }
}
