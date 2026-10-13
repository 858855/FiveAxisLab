#include "compute.h"
#include "compute_abi.h"
#include <QCoreApplication>
#include <QDir>
#include <QElapsedTimer>
#include <QLibrary>
#include <algorithm>
#include <condition_variable>
#include <deque>
#include <future>
#include <mutex>
#include <stdexcept>
#include <thread>
namespace fiveaxis {
namespace {
thread_local bool inPool = false;
class Pool {
    std::mutex mutex;
    std::condition_variable wake;
    bool stop = false;
    std::deque<std::function<void()>> queue;
    std::vector<std::thread> workers;

  public:
    Pool() {
        for (unsigned i = 0; i < std::max(1u, std::thread::hardware_concurrency()); ++i)
            workers.emplace_back([this] {
                inPool = true;
                for (;;) {
                    std::function<void()> work;
                    {
                        std::unique_lock<std::mutex> lock(mutex);
                        wake.wait(lock, [&] { return stop || !queue.empty(); });
                        if (stop && queue.empty())
                            return;
                        work = std::move(queue.front());
                        queue.pop_front();
                    }
                    work();
                }
            });
    }
    ~Pool() {
        {
            std::lock_guard<std::mutex> lock(mutex);
            stop = true;
        }
        wake.notify_all();
        for (auto &worker : workers)
            worker.join();
    }
    std::future<void> submit(std::function<void()> work) {
        auto task = std::make_shared<std::packaged_task<void()>>(std::move(work));
        auto result = task->get_future();
        {
            std::lock_guard<std::mutex> lock(mutex);
            queue.emplace_back([task] { (*task)(); });
        }
        wake.notify_one();
        return result;
    }
};
Pool &pool() {
    static Pool value;
    return value;
}
} // namespace
int compute_threads(const ComputeSettings &s) {
    int available = std::max(1, int(std::thread::hardware_concurrency()));
    return s.threads > 0 ? std::clamp(s.threads, 1, available) : std::max(1, available - 1);
}
void parallel_for(size_t count, const ComputeSettings &settings, const std::function<void(size_t)> &task,
                  std::atomic_bool *cancel) {
    int workers = std::min<size_t>(compute_threads(settings), count);
    if (inPool || workers <= 1) {
        for (size_t i = 0; i < count; ++i) {
            if (cancel && cancel->load())
                throw std::runtime_error("Job cancelled");
            task(i);
        }
        return;
    }
    std::atomic<size_t> next{0};
    std::vector<std::future<void>> futures;
    for (int w = 0; w < workers; ++w)
        futures.push_back(pool().submit([&] {
            for (;;) {
                if (cancel && cancel->load())
                    return;
                size_t i = next.fetch_add(1);
                if (i >= count)
                    return;
                task(i);
            }
        }));
    std::exception_ptr error;
    for (auto &future : futures)
        try {
            future.get();
        } catch (...) {
            if (!error)
                error = std::current_exception();
        }
    if (error)
        std::rethrow_exception(error);
    if (cancel && cancel->load())
        throw std::runtime_error("Job cancelled");
}
static QString plugin_path(const QString &name) {
    return QDir(QCoreApplication::applicationDirPath()).filePath("compute/" + name);
}
QStringList compute_devices() {
    QStringList result{"CPU"};
    for (auto name : {"fal_cuda", "fal_vulkan"}) {
        QLibrary library(plugin_path(name));
        if (!library.load())
            continue;
        auto api = reinterpret_cast<FalGetApi>(library.resolve("fal_get_api"));
        if (!api)
            continue;
        auto functions = api(FAL_COMPUTE_ABI);
        if (!functions)
            continue;
        for (int i = 0; i < functions->device_count(); ++i) {
            char description[256] = {};
            if (functions->device_name(i, description, sizeof(description)) == 0)
                result << QString(name == QString("fal_cuda") ? "CUDA: " : "Vulkan: ") +
                              QString::fromUtf8(description);
        }
    }
    return result;
}
std::vector<Vec3> compute_workload(const std::vector<double> &input, uint32_t workload,
                                   const Transform &transform, const ComputeSettings &settings,
                                   ComputeReport &report, std::atomic_bool *cancel) {
    size_t stride = workload == 2 ? 16 : workload == 3 ? 9 : 3;
    size_t count = input.size() / stride;
    if (workload > 3 || input.size() % stride)
        throw std::runtime_error("Invalid compute workload buffer");
    for (double value : input)
        if (!std::isfinite(value))
            throw std::runtime_error("Non-finite compute input");
    if (!transform.matrix().allFinite())
        throw std::runtime_error("Non-finite transform");
    auto reference = [&](size_t i) {
        if (workload == 3) {
            Vec3 a(input[i * 9], input[i * 9 + 1], input[i * 9 + 2]),
                b(input[i * 9 + 3], input[i * 9 + 4], input[i * 9 + 5]),
                c(input[i * 9 + 6], input[i * 9 + 7], input[i * 9 + 8]);
            Vec3 n = (b - a).cross(c - a);
            return n.norm() > 1e-12 ? Vec3(n.normalized()) : Vec3::Zero();
        }
        if (workload == 0)
            return Vec3(transform * Vec3(input[i * 3], input[i * 3 + 1], input[i * 3 + 2]));
        if (workload == 1)
            return Vec3(input[i * 3], input[i * 3 + 1],
                        input[i * 3] <= input[i * 3 + 2] && input[i * 3 + 1] >= input[i * 3 + 2] ? 1. : 0.);
        Vec3 value = Vec3::Zero();
        for (int n = 0; n < 4; ++n)
            for (int c = 0; c < 3; ++c)
                value[c] += input[i * 16 + n] * input[i * 16 + 4 + n * 3 + c];
        return value;
    };
    QElapsedTimer timer;
    timer.start();
    std::vector<Vec3> output(count);
    report = ComputeReport{};
    auto cpu = [&] { parallel_for(count, settings, [&](size_t i) { output[i] = reference(i); }, cancel); };
    cpu();
    for (const auto &value : output)
        if (!value.allFinite())
            throw std::runtime_error("Compute result overflow");
    report.cpuMilliseconds = timer.nsecsElapsed() / 1e6;
    if (count == 0 || settings.backend == ComputeBackend::CPU ||
        (count < 4096 && settings.backend == ComputeBackend::Auto)) {
        if (settings.backend == ComputeBackend::Auto)
            report.fallback = "Small batch uses CPU";
        report.milliseconds = timer.nsecsElapsed() / 1e6;
        return output;
    }
    bool used = false;
    static std::mutex dispatchMutex;
    std::lock_guard<std::mutex> dispatch(dispatchMutex);
    static std::map<QString, bool> slower;
    for (auto backend : {ComputeBackend::CUDA, ComputeBackend::Vulkan}) {
        if (settings.backend == ComputeBackend::CPU ||
            (settings.backend != ComputeBackend::Auto && settings.backend != backend))
            continue;
        QLibrary library(plugin_path(backend == ComputeBackend::CUDA ? "fal_cuda" : "fal_vulkan"));
        if (!library.load()) {
            report.fallback = library.errorString();
            continue;
        }
        auto get = reinterpret_cast<FalGetApi>(library.resolve("fal_get_api"));
        const auto *api = get ? get(FAL_COMPUTE_ABI) : nullptr;
        if (!api) {
            report.fallback = "Compute ABI mismatch";
            continue;
        }
        int device = -1;
        for (int i = 0; i < api->device_count(); ++i) {
            char description[256] = {};
            if (api->device_name(i, description, sizeof(description)) != 0)
                continue;
            if (settings.device.isEmpty() || settings.device == QString::fromUtf8(description)) {
                device = i;
                report.device = QString::fromUtf8(description);
                break;
            }
        }
        if (device < 0) {
            report.fallback = "Requested GPU unavailable";
            continue;
        }
        QString timingKey = QString::number(workload) + "/" + QString::number(int(backend)) + "/" +
                            report.device + "/" + QString::number(int(std::log2(std::max(size_t(1), count))));
        if (settings.backend == ComputeBackend::Auto && slower[timingKey]) {
            report.fallback =
                "Previous end-to-end measurement favored CPU for this workload/device/batch size";
            continue;
        }
        double matrix[12];
        for (int r = 0; r < 3; ++r)
            for (int c = 0; c < 4; ++c)
                matrix[r * 4 + c] = transform.matrix()(r, c);
        QElapsedTimer gpuTimer;
        gpuTimer.start();
        std::vector<double> values(count * 3);
        char error[256] = {};
        void *context = api->create(device, settings.memoryFraction, error, sizeof(error));
        if (!context) {
            report.fallback = QString::fromUtf8(error);
            continue;
        }
        bool success = true;
        size_t batch = 65536;
        for (size_t begin = 0; begin < count; begin += batch) {
            if (cancel && cancel->load()) {
                api->destroy(context);
                throw std::runtime_error("GPU job cancelled");
            }
            size_t chunk = std::min(batch, count - begin);
            if (api->transform(context, input.data() + stride * begin, chunk, matrix,
                               values.data() + 3 * begin, error, sizeof(error), workload) != 0) {
                success = false;
                report.fallback = QString::fromUtf8(error);
                break;
            }
            ++report.batches;
        }
        api->destroy(context);
        report.gpuMilliseconds = gpuTimer.nsecsElapsed() / 1e6;
        if (!success)
            continue;
        // Certification is CPU-owned: compare every result to FP64 reference. Device errors
        // cannot propagate into topology, placement, or a clearance certificate.
        for (size_t i = 0; i < count; ++i) {
            Vec3 expected = output[i];
            Vec3 value(values[i * 3], values[i * 3 + 1], values[i * 3 + 2]);
            if (!value.allFinite() || (workload == 1 && value.z() != expected.z()) ||
                (value - expected).norm() > 1e-10 * std::max(1., expected.norm())) {
                success = false;
                report.fallback = "GPU numerical parity failed";
                break;
            }
        }
        if (success)
            slower[timingKey] = timer.nsecsElapsed() / 1e6 >= report.cpuMilliseconds;
        if (success && settings.backend == ComputeBackend::Auto && slower[timingKey]) {
            slower[timingKey] = true;
            report.fallback = "Measured GPU time including transfers exceeds CPU; Auto retains CPU";
            break;
        }
        if (success) {
            used = true;
            report.backend = backend == ComputeBackend::CUDA ? "CUDA" : "Vulkan";
            report.precision = "FP64 with CPU verification";
            report.fallback.clear();
            break;
        }
    }
    if (!used) {
        report.backend = "CPU";
        report.device = "CPU";
    }
    report.milliseconds = timer.nsecsElapsed() / 1e6;
    return output;
}
std::vector<Vec3> compute_transform(const std::vector<Vec3> &points, const Transform &transform,
                                    const ComputeSettings &settings, ComputeReport &report,
                                    std::atomic_bool *cancel) {
    std::vector<double> input;
    input.reserve(points.size() * 3);
    for (const auto &point : points)
        for (int c = 0; c < 3; ++c)
            input.push_back(point[c]);
    return compute_workload(input, 0, transform, settings, report, cancel);
}
} // namespace fiveaxis
