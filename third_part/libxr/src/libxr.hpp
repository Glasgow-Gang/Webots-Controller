#pragma once

#include "libxr_assert.hpp"
#include "libxr_cb.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_string.hpp"
#include "libxr_system.hpp"
#include "libxr_time.hpp"
#include "libxr_type.hpp"
#include "lock_queue.hpp"
#include "message.hpp"
#include "mutex.hpp"
#include "queue.hpp"
#include "ramfs.hpp"
#include "semaphore.hpp"
#include "signal.hpp"
#include "terminal.hpp"
#include "thread.hpp"
#include "timer.hpp"

namespace LibXR {
void LibXR_Init();
}