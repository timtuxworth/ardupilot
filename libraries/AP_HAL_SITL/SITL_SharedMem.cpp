#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include "SITL_SharedMem.h"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <signal.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>

AP_SITL_SharedMem::AP_SITL_SharedMem() :
    _data(nullptr),
    _fd(-1),
    _instance_id(0),
    _created(false),
    _waiting_announced(false),
    _sync_announced(false)
{
    memset(_peer_seen_time_us, 0, sizeof(_peer_seen_time_us));
    memset(_peer_seen_wall_us, 0, sizeof(_peer_seen_wall_us));
    memset(_peer_stalled, 0, sizeof(_peer_stalled));
    memset(_peer_resume_announced, 0, sizeof(_peer_resume_announced));
    memset(_peer_cooldown_until_us, 0, sizeof(_peer_cooldown_until_us));
    _catchup_started_wall_us = 0;
    _catchup_gave_up = false;

    _process_start_wall_us = _now_us();
    _instant_catchup_done = false;
}

AP_SITL_SharedMem::~AP_SITL_SharedMem()
{
    _cleanup();
}

uint64_t AP_SITL_SharedMem::_now_us()
{
    struct timeval tv;
    gettimeofday(&tv, nullptr);
    return (uint64_t)tv.tv_sec * 1000000ULL + (uint64_t)tv.tv_usec;
}

// highest sim_time_us reported by any live peer among the first n slots
uint64_t AP_SITL_SharedMem::_max_peer_time(uint32_t n) const
{
    uint64_t max_peer_time = 0;
    for (uint32_t i = 0; i < n; i++) {
        if (i == _instance_id) {
            continue;
        }
        const pid_t pid = _data->instance[i].pid;
        if (pid <= 0 || kill(pid, 0) != 0) {
            continue;
        }
        const uint64_t peer_time = _data->instance[i].sim_time_us;
        if (peer_time > max_peer_time) {
            max_peer_time = peer_time;
        }
    }
    return max_peer_time;
}

bool AP_SITL_SharedMem::init(uint8_t instance_id, uint8_t total_instances)
{
    if (instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        if (total_instances > 1) fprintf(stderr, "SITL_SharedMem: instance_id %u >= max %u\n",
                                         instance_id, AP_SITL_SHMEM_MAX_INSTANCES);
        return false;
    }

    _instance_id = instance_id;

    // try to open existing segment first
    _fd = shm_open(AP_SITL_SHMEM_NAME, O_RDWR | O_CREAT | O_EXCL,
                   S_IRUSR | S_IWUSR);
    if (_fd >= 0) {
        // we created the segment - initialise it
        _created = true;
        if (ftruncate(_fd, sizeof(AP_SITL_ShmData)) < 0) {
            if (total_instances > 1) perror("SITL_SharedMem: ftruncate");
            _cleanup();
            return false;
        }
    } else {
        // segment exists - open it
        _fd = shm_open(AP_SITL_SHMEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
        if (_fd < 0) {
            if (total_instances > 1) perror("SITL_SharedMem: shm_open");
            return false;
        }
        _created = false;

        // A stale/undersized leftover segment could SIGBUS us when we
        // touch pages beyond its backing size. Detect that and replace
        // the segment rather than crashing later.
        struct stat st;
        if (fstat(_fd, &st) < 0) {
            if (total_instances > 1) perror("SITL_SharedMem: fstat");
            _cleanup();
            return false;
        }
        if ((size_t)st.st_size < sizeof(AP_SITL_ShmData)) {
            if (total_instances > 1) fprintf(stderr,
                                             "SITL_SharedMem: stale/undersized segment %s "
                                             "(%lld bytes, need %zu) - recreating\n",
                                             AP_SITL_SHMEM_NAME, (long long)st.st_size,
                                             sizeof(AP_SITL_ShmData));
            close(_fd);
            shm_unlink(AP_SITL_SHMEM_NAME);
            _fd = shm_open(AP_SITL_SHMEM_NAME, O_RDWR | O_CREAT | O_EXCL,
                           S_IRUSR | S_IWUSR);
            if (_fd < 0) {
                // lost a race with another instance also recreating it;
                // just open whatever is there now and trust its size
                _fd = shm_open(AP_SITL_SHMEM_NAME, O_RDWR, S_IRUSR | S_IWUSR);
                if (_fd < 0) {
                    if (total_instances > 1) perror("SITL_SharedMem: shm_open (recreate)");
                    return false;
                }
            } else {
                _created = true;
                if (ftruncate(_fd, sizeof(AP_SITL_ShmData)) < 0) {
                    if (total_instances > 1) perror("SITL_SharedMem: ftruncate (recreate)");
                    _cleanup();
                    return false;
                }
            }
        }
    }

    _data = static_cast<AP_SITL_ShmData *>(
        mmap(nullptr, sizeof(AP_SITL_ShmData),
             PROT_READ | PROT_WRITE, MAP_SHARED, _fd, 0));

    if (_data == MAP_FAILED) {
        if (total_instances > 1) perror("SITL_SharedMem: mmap");
        _data = nullptr;
        _cleanup();
        return false;
    }

    if (_created) {
        memset(_data, 0, sizeof(AP_SITL_ShmData));
        _data->magic   = AP_SITL_SHMEM_MAGIC;
        _data->version = 1;

        // initialise each slot's payload mutex as a process-shared
        // (and, where supported, robust) mutex - memset() alone is not
        // a valid pthread_mutex_t.
        pthread_mutexattr_t attr;
        pthread_mutexattr_init(&attr);
        pthread_mutexattr_setpshared(&attr, PTHREAD_PROCESS_SHARED);
#ifdef PTHREAD_MUTEX_ROBUST
        pthread_mutexattr_setrobust(&attr, PTHREAD_MUTEX_ROBUST);
#endif
        for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
            pthread_mutex_init(&_data->instance[i].payload_mutex, &attr);
        }
        pthread_mutexattr_destroy(&attr);
    } else if (_data->magic != AP_SITL_SHMEM_MAGIC) {
        if (total_instances > 1) fprintf(stderr, "SITL_SharedMem: bad magic 0x%08X in existing segment\n",
                                         _data->magic);
        _cleanup();
        return false;
    }

    // publish total_instances: use max of current value and what we know
    if (total_instances > _data->total_instances) {
        _data->total_instances = total_instances;
    }

    // claim our slot
    _data->instance[_instance_id].pid         = getpid();
    _data->instance[_instance_id].sim_time_us = 0;

    return true;
}

void AP_SITL_SharedMem::update(uint64_t time_us)
{
    if (_data == nullptr) {
        return;
    }
    _data->instance[_instance_id].sim_time_us = time_us;
}

uint64_t AP_SITL_SharedMem::get_time_us(uint8_t instance_id) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return 0;
    }
    return _data->instance[instance_id].sim_time_us;
}

bool AP_SITL_SharedMem::instance_active(uint8_t instance_id) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }
    const pid_t pid = _data->instance[instance_id].pid;
    if (pid <= 0) {
        return false;
    }
    // check pid is alive by sending signal 0
    return kill(pid, 0) == 0;
}

uint8_t AP_SITL_SharedMem::get_instance_count() const
{
    if (_data == nullptr) {
        return 0;
    }
    uint8_t count = 0;
    for (uint8_t i = 0; i < AP_SITL_SHMEM_MAX_INSTANCES; i++) {
        if (instance_active(i)) {
            count++;
        }
    }
    return count;
}

/*
  write our payload block while holding the slot's robust process-shared
  mutex. EOWNERDEAD (a prior crashed run reusing this slot) is marked
  consistent and we proceed, since we are the sole writer for our slot.
*/
void AP_SITL_SharedMem::write_payload(const void *data, uint32_t len)
{
    if (_data == nullptr) {
        return;
    }
    if (len > AP_SITL_SHMEM_PAYLOAD_SIZE) {
        len = AP_SITL_SHMEM_PAYLOAD_SIZE;
    }
    auto &slot = _data->instance[_instance_id];

    const int ret = pthread_mutex_lock(&slot.payload_mutex);
#ifdef PTHREAD_MUTEX_ROBUST
    if (ret == EOWNERDEAD) {
        pthread_mutex_consistent(&slot.payload_mutex);
    } else
#endif
    if (ret != 0) {
        if (_data->total_instances > 1) fprintf(stderr, "SITL_SharedMem: write_payload lock failed: %d\n", ret);
        return;
    }

    memcpy(slot.payload, data, len);

    pthread_mutex_unlock(&slot.payload_mutex);
}

/*
  read a peer's payload block while holding its robust process-shared
  mutex. EOWNERDEAD means the peer died mid-write; mark it consistent
  but treat the data as unreliable and return false.
*/
bool AP_SITL_SharedMem::read_payload(uint8_t instance_id, void *data, uint32_t len) const
{
    if (_data == nullptr || instance_id >= AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }
    if (len > AP_SITL_SHMEM_PAYLOAD_SIZE) {
        len = AP_SITL_SHMEM_PAYLOAD_SIZE;
    }
    auto &slot = _data->instance[instance_id];

    const int ret = pthread_mutex_lock(&slot.payload_mutex);
    bool owner_died = false;
#ifdef PTHREAD_MUTEX_ROBUST
    if (ret == EOWNERDEAD) {
        owner_died = true;
        pthread_mutex_consistent(&slot.payload_mutex);
    } else
#endif
    if (ret != 0) {
        if (_data->total_instances > 1) fprintf(stderr, "SITL_SharedMem: read_payload lock failed: %d\n", ret);
        return false;
    }

    if (!owner_died) {
        memcpy(data, slot.payload, len);
    }

    pthread_mutex_unlock(&slot.payload_mutex);

    return !owner_died;
}


/*
  spin-wait barrier: pause until all expected peer instances have
  published a sim_time_us >= (sim_time_us - max_skew_us), keeping all
  instances within max_skew_us of each other. Called from
  Aircraft::sync_frame_time() after every simulation step. A wall-clock
  timeout (default 5s) prevents an indefinite stall when a peer crashes.
*/
bool AP_SITL_SharedMem::sync_with_peers(uint64_t sim_time_us,
                                         uint64_t max_skew_us,
                                         uint64_t timeout_us,
                                         uint64_t stall_grace_us)
{
    if (_data == nullptr || _data->total_instances <= 1) {
        return true;
    }

    const uint32_t n = _data->total_instances;
    if (n > AP_SITL_SHMEM_MAX_INSTANCES) {
        return true;
    }

    // minimum sim_time we are willing to be ahead of peers
    const uint64_t min_peer_time = (sim_time_us > max_skew_us)
                                    ? sim_time_us - max_skew_us
                                    : 0;

    // wall-clock start, for the overall timeout
    const uint64_t start_us = _now_us();

    while (true) {
        // wall-clock "now", used both for the overall timeout and for
        // per-peer stall detection below
        const uint64_t now_us = _now_us();

        bool all_ok = true;
        for (uint32_t i = 0; i < n; i++) {
            if (i == _instance_id) {
                continue;
            }
            const pid_t pid = _data->instance[i].pid;
            if (pid == 0) {
                // slot not yet registered; wait for it
                if (!_waiting_announced) {
                    _waiting_announced = true;
                    fprintf(stderr,
                            "SITL_SharedMem: instance %u waiting for all %u "
                            "instances to register...\n",
                            (unsigned)_instance_id, (unsigned)n);
                }
                all_ok = false;
                break;
            }
            if (pid < 0) {
                // peer registered earlier but has died; don't wait for it
                continue;
            }
            // if the peer has crashed, stop waiting for it
            if (kill(pid, 0) != 0) {
                // mark the slot dead (-1, not 0) so we don't mistake it
                // for "not yet registered" and stall on it forever
                _data->instance[i].pid = -1;
                fprintf(stderr,
                        "SITL_SharedMem: peer instance %u (pid %d) has gone "
                        "away; %u of %u instances remain\n",
                        (unsigned)i, (int)pid,
                        (unsigned)get_instance_count(), (unsigned)n);
                continue;
            }

            // track whether this peer's sim_time_us is still advancing;
            // after stall_grace_us with no movement we stop waiting on
            // it. A reboot resets sim_time_us to 0 - that backward jump
            // must not count as progress or it would restart the timer.
            const uint64_t peer_time = _data->instance[i].sim_time_us;
            bool peer_advanced = false;
            if (peer_time > _peer_seen_time_us[i]) {
                peer_advanced = true;
                _peer_seen_time_us[i] = peer_time;
                _peer_seen_wall_us[i] = now_us;
            } else if (peer_time != _peer_seen_time_us[i]) {
                // backward jump (e.g. reboot reset) - remember the new
                // value for future comparisons, but don't treat it as
                // progress or reset the grace-period clock
                _peer_seen_time_us[i] = peer_time;
            }

            if (_peer_stalled[i]) {
                // Only rejoin once genuinely caught up (matches the
                // threshold is_behind_peers() uses to keep sprinting).
                // The cooldown below, not a wider margin, prevents flapping.
                if (peer_time >= min_peer_time) {
                    _peer_stalled[i] = false;
                    _peer_resume_announced[i] = false;
                    // brief cooldown so a scheduling hiccup right after
                    // rejoining doesn't immediately re-exclude the peer
                    _peer_cooldown_until_us[i] = now_us + 2 * stall_grace_us;
                    fprintf(stderr,
                            "SITL_SharedMem: peer instance %u has "
                            "caught-up to the shared sim time\n", (unsigned)i);
                } else {
                    if (peer_advanced && !_peer_resume_announced[i]) {
                        _peer_resume_announced[i] = true;
                        fprintf(stderr,
                                "SITL_SharedMem: peer instance %u resumed, "
                                "still catching up\n", (unsigned)i);
                    }
                    continue;
                }
            } else if (peer_time < min_peer_time &&
                       now_us - _peer_seen_wall_us[i] > stall_grace_us &&
                       now_us >= _peer_cooldown_until_us[i]) {
                // only stall a peer that is actually behind (blocking us);
                // a peer that simply updates less often than we poll but
                // is still within skew must never be marked stalled, or it
                // will instantly "rejoin" and flap forever
                _peer_stalled[i] = true;
                _peer_resume_announced[i] = false;
                fprintf(stderr,
                        "SITL_SharedMem: peer instance %u stalled "
                        "(%.2fs); no longer blocking swarm\n",
                        (unsigned)i, stall_grace_us * 1e-6);
                continue;
            }

            if (_data->instance[i].sim_time_us < min_peer_time) {
                all_ok = false;
                break;
            }
        }
        if (all_ok) {
            if (!_sync_announced) {
                _sync_announced = true;
                fprintf(stderr,
                        "SITL_SharedMem: instance %u in lock-step with %u "
                        "instances (max skew %llu us)\n",
                        (unsigned)_instance_id, (unsigned)n,
                        (unsigned long long)max_skew_us);
            }
            return true;
        }

        // check wall-clock timeout
        const uint64_t elapsed_us = now_us - start_us;
        if (elapsed_us > timeout_us) {
            fprintf(stderr,
                    "SITL_SharedMem: sync timeout after %.1f s waiting for "
                    "peers (our sim_time_us=%llu)\n",
                    elapsed_us * 1e-6,
                    (unsigned long long)sim_time_us);
            return false;
        }

        // short sleep to avoid hammering the cache line
        usleep(100);
    }
}

/*
  return true if our own sim_time_us is more than max_skew_us behind the
  most-advanced live peer - used to trigger a ~2x-speedup catch-up
  sprint (see Aircraft::sync_frame_time()). The sprint is capped at
  max_catchup_us of wall-clock time, after which we give up and run at
  normal speed (still excluded from the barrier by sync_with_peers()).
*/
bool AP_SITL_SharedMem::is_behind_peers(uint64_t sim_time_us, uint64_t max_skew_us,
                                         uint64_t max_catchup_us)
{
    if (_data == nullptr || _data->total_instances <= 1) {
        return false;
    }

    const uint32_t n = _data->total_instances;
    if (n > AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }

    const uint64_t max_peer_time = _max_peer_time(n);
    const bool behind = sim_time_us + max_skew_us < max_peer_time;
    const uint64_t now_us = _now_us();

    if (!behind) {
        // caught up (or never fell behind) - reset catch-up tracking so a
        // future reboot starts a fresh sprint window
        _catchup_started_wall_us = 0;
        _catchup_gave_up = false;
        return false;
    }

    if (_catchup_gave_up) {
        // already gave up sprinting this episode; stay at normal speed
        // until we catch up naturally (behind will go false above once
        // we do, which resets _catchup_gave_up for next time)
        return false;
    }

    if (_catchup_started_wall_us == 0) {
        _catchup_started_wall_us = now_us;
    } else if (now_us - _catchup_started_wall_us > max_catchup_us) {
        _catchup_gave_up = true;
        fprintf(stderr,
                "SITL_SharedMem: instance %u could not catch up to peers "
                "within %.1fs; giving up sprint, running at normal speed\n",
                (unsigned)_instance_id, max_catchup_us * 1e-6);
        return false;
    }

    return true;
}

bool AP_SITL_SharedMem::instant_catchup_if_new(uint64_t &sim_time_us, uint64_t max_uptime_us)
{
    if (_instant_catchup_done || _data == nullptr || _data->total_instances <= 1) {
        return false;
    }

    const uint64_t now_us = _now_us();
    if (now_us - _process_start_wall_us > max_uptime_us) {
        // too long since our own process started - not a fresh reboot
        // any more, don't instantly jump the clock
        _instant_catchup_done = true;
        return false;
    }

    const uint32_t n = _data->total_instances;
    if (n > AP_SITL_SHMEM_MAX_INSTANCES) {
        return false;
    }

    const uint64_t max_peer_time = _max_peer_time(n);
    if (max_peer_time <= sim_time_us) {
        // not behind anyone - nothing to do, but don't consume our one
        // shot in case we fall behind moments later during startup
        return false;
    }

    _instant_catchup_done = true;
    fprintf(stderr,
            "SITL_SharedMem: instance %u fresh reboot, jumping clock to "
            "match peers (%.2fs ahead)\n",
            (unsigned)_instance_id, (max_peer_time - sim_time_us) * 1e-6);
    sim_time_us = max_peer_time;
    return true;
}

void AP_SITL_SharedMem::_clear_slot()
{
    if (_data == nullptr) {
        return;
    }
    _data->instance[_instance_id].sim_time_us = 0;
    // -1 (not 0) so peers see "registered but gone" rather than
    // "not yet registered", which they would wait for indefinitely
    _data->instance[_instance_id].pid         = -1;
}

void AP_SITL_SharedMem::_cleanup()
{
    _clear_slot();

    if (_data != nullptr && _data != MAP_FAILED) {
        munmap(_data, sizeof(AP_SITL_ShmData));
        _data = nullptr;
    }

    if (_fd >= 0) {
        close(_fd);
        _fd = -1;
    }

    // only the creator tries to unlink; if other instances are still
    // alive it is harmless because they hold their own mappings
    if (_created) {
        shm_unlink(AP_SITL_SHMEM_NAME);
        _created = false;
    }
}

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
