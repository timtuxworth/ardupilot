#pragma once

#include <AP_HAL/AP_HAL.h>

#if CONFIG_HAL_BOARD == HAL_BOARD_SITL

#include <stdint.h>
#include <pthread.h>

/*
  shared memory segment allowing multiple SITL instances to share
  simulated clock state. Each instance writes its own sim_time_us into
  a slot indexed by instance number; readers check peer times to
  determine relative clock positions. Named /ardupilot_sitl_shmem.

  The clock field needs no locking (single writer, atomic 64-bit write).
  The payload[] block is control-critical (e.g. shared vehicle telemetry)
  so it's protected by a process-shared, robust pthread mutex instead -
  see write_payload()/read_payload().
*/

#define AP_SITL_SHMEM_MAX_INSTANCES 16
#define AP_SITL_SHMEM_MAGIC         0x4150534E  // "APSN" (v2 layout, bumped from "APSM")
#define AP_SITL_SHMEM_NAME          "/ardupilot_sitl_shmem"

// size of the arbitrary per-instance data block, in addition to the clock.
// e.g. vehicle position/attitude or other user-defined telemetry.
#define AP_SITL_SHMEM_PAYLOAD_SIZE  4096

/*
  layout of the shared memory segment
*/
struct AP_SITL_ShmData {
    uint32_t magic;
    uint32_t version;
    // number of instances expected in this session; set by each registering
    // instance.  The highest value written wins, so all instances must agree
    // on the fleet size before calling sync_with_peers().
    uint32_t total_instances;
    uint32_t _pad;
    struct {
        uint64_t sim_time_us;   // simulated time in microseconds
        pid_t    pid;           // pid of owning process (0 = never used,
                                // -1 = peer registered but has since died)
        uint32_t _pad;
        // process-shared robust mutex guarding payload[]. Robust means
        // a locker that dies mid-hold gives the next locker EOWNERDEAD
        // instead of hanging - see write_payload()/read_payload().
        pthread_mutex_t payload_mutex;
        uint8_t  payload[AP_SITL_SHMEM_PAYLOAD_SIZE];
    } instance[AP_SITL_SHMEM_MAX_INSTANCES];
};

class AP_SITL_SharedMem {
public:
    AP_SITL_SharedMem();
    ~AP_SITL_SharedMem();

    /*
      initialise shared memory for the given instance number.
      total_instances is the total fleet size (e.g. 3 for --instance 0/1/2).
      Must be called once on startup before update() or get_time_us().
    */
    bool init(uint8_t instance_id, uint8_t total_instances = 1);

    /*
      update this instance's clock in the shared segment.
      Call this periodically (e.g. every scheduler tick).
    */
    void update(uint64_t time_us);

    /*
      return the last published sim_time_us for the given instance.
      Returns 0 if the instance has no active entry or shm is not initialised.
    */
    uint64_t get_time_us(uint8_t instance_id) const;

    /*
      return true if the given instance slot appears active
      (pid is alive and time is non-zero)
    */
    bool instance_active(uint8_t instance_id) const;

    /*
      return the number of currently active instances
    */
    uint8_t get_instance_count() const;

    /*
      write up to AP_SITL_SHMEM_PAYLOAD_SIZE bytes into this instance's
      payload block, for peers to read via read_payload(). Takes the
      slot's robust process-shared mutex for the duration of the copy so
      readers never observe a torn write.
    */
    void write_payload(const void *data, uint32_t len);

    /*
      read the given instance's payload block into data (up to len bytes,
      capped at AP_SITL_SHMEM_PAYLOAD_SIZE). Takes the slot's robust
      process-shared mutex for the duration of the copy. Returns true on
      success, false if shm is not initialised or instance_id is out of
      range.
    */
    bool read_payload(uint8_t instance_id, void *data, uint32_t len) const;

    /*
      barrier sync: spin-wait until all expected peer instances have
      published a sim_time_us >= (sim_time_us - max_skew_us). Returns
      false on timeout_us (wall-clock us) expiry, e.g. a peer crashed.

      A peer whose sim_time_us stalls (e.g. mid-reboot) but is still
      alive is excluded from the barrier after stall_grace_us, so it
      doesn't block the rest of the swarm for the full timeout_us.
    */
    bool sync_with_peers(uint64_t sim_time_us,
                         uint64_t max_skew_us = 5000,
                         uint64_t timeout_us = 5000000,
                         uint64_t stall_grace_us = 300000);

    /*
      return true if our own sim_time_us is more than max_skew_us behind
      the most-advanced live peer - used to trigger a ~2x-speedup
      catch-up sprint (see Aircraft::sync_frame_time()) so a rebooted
      instance fast-forwards back into lock-step instead of remaining
      permanently excluded from the barrier.

      The sprint is capped at max_catchup_us of wall-clock time; once
      exceeded we give up and return false, running at normal speed
      while sync_with_peers() still excludes us from the barrier.
    */
    bool is_behind_peers(uint64_t sim_time_us, uint64_t max_skew_us = 5000,
                          uint64_t max_catchup_us = 60000000);

    /*
      one-shot: if this process started less than max_uptime_us ago and
      is behind live peers, snap sim_time_us to match them instantly
      (no sprint) and return true. Meant for a just-rebooted instance
      to rejoin lock-step immediately rather than sprinting to catch up.
    */
    bool instant_catchup_if_new(uint64_t &sim_time_us, uint64_t max_uptime_us = 10000000);

    bool is_initialised() const { return _data != nullptr; }
    bool is_multi_instance() const { return _data != nullptr && _data->total_instances > 1; }

private:
    AP_SITL_ShmData *_data;
    int              _fd;
    uint8_t          _instance_id;
    bool             _created;  // true if we created the shm segment
    bool             _waiting_announced;  // printed "waiting for peers" once
    bool             _sync_announced;     // printed "in lock-step" once

    // per-peer stall tracking for sync_with_peers(): last sim_time_us
    // observed per peer, when it last changed, and stall/resume state.
    uint64_t         _peer_seen_time_us[AP_SITL_SHMEM_MAX_INSTANCES];
    uint64_t         _peer_seen_wall_us[AP_SITL_SHMEM_MAX_INSTANCES];
    bool             _peer_stalled[AP_SITL_SHMEM_MAX_INSTANCES];
    bool             _peer_resume_announced[AP_SITL_SHMEM_MAX_INSTANCES];  // printed "resumed, still catching up" once per stall

    // wall-clock time before which a just-rejoined peer may not be
    // re-marked stalled; prevents flapping right at the min_peer_time
    // boundary under CPU contention from sprinting instances.
    uint64_t         _peer_cooldown_until_us[AP_SITL_SHMEM_MAX_INSTANCES];

    // is_behind_peers() catch-up-sprint tracking: the wall-clock time we
    // first noticed we were behind, and whether we've since given up
    // sprinting (having hit max_catchup_us) until we catch up naturally.
    uint64_t         _catchup_started_wall_us;
    bool             _catchup_gave_up;

    // instant_catchup_if_new(): wall-clock time this process started, and
    // whether we've already taken (or forfeited) our one-shot clock jump.
    uint64_t         _process_start_wall_us;
    bool             _instant_catchup_done;

    static uint64_t _now_us();
    uint64_t _max_peer_time(uint32_t n) const;

    void _clear_slot();
    void _cleanup();
};

#endif // CONFIG_HAL_BOARD == HAL_BOARD_SITL
