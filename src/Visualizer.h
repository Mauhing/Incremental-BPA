#ifndef VISUALIZER_H
#define VISUALIZER_H

#include <mutex>
#include <condition_variable>
#include <atomic>
#include "Mesher.h"

namespace Visualizer
{
    void visualizationThread(
        Mesher& mesher,
        std::mutex& o3d_mesh_mutex,
        std::condition_variable& vis_cv,
        std::atomic<bool>& should_exit
    );
}

#endif // VISUALIZER_H