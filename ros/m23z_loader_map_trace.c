/*
 * M23R: bounded, opt-in dynamic-loader lifetime trace for the existing
 * no-device image_proc component_container. Diagnostic only.
 *
 * LD_PRELOAD is set by the M23R launch description only on the
 * component_container child, not on the synthetic publisher/observer.
 * No device opens, no signals, no interposition on DDS or ROS APIs.
 * Interposition is potentially timing-sensitive: absence of a crash
 * under tracing is never evidence the underlying defect was fixed.
 */
#define _GNU_SOURCE
#include <dlfcn.h>
#include <elf.h>
#include <stdint.h>
#include <link.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/syscall.h>
#include <time.h>
#include <unistd.h>

static void *(*next_dlopen)(const char *, int) = NULL;
static int (*next_dlclose)(void *) = NULL;
static int enabled = 0;
static __thread int in_trace = 0;

static int is_component_container(void) {
    char exe[512];
    ssize_t n = readlink("/proc/self/exe", exe, sizeof(exe) - 1);
    if (n <= 0 || n >= (ssize_t)sizeof(exe) - 1) {
        return 0;
    }
    exe[n] = '\0';
    const char *name = strrchr(exe, '/');
    name = name ? name + 1 : exe;
    return strcmp(name, "component_container") == 0;
}

static void event(const char *stage, const char *module, void *handle, int status) {
    if (!enabled || in_trace) {
        return;
    }
    in_trace = 1;
    struct timespec now = {0, 0};
    (void)clock_gettime(CLOCK_MONOTONIC, &now);
    char buffer[768];
    const char *safe_module = (module && module[0]) ? module : "<unnamed>";
    int n = snprintf(buffer, sizeof(buffer),
                     "M23R_TRACE_EVENT=%s mono_ns=%lld pid=%ld tid=%ld "
                     "handle=%p status=%d module=%.400s\n",
                     stage,
                     (long long)now.tv_sec * 1000000000LL + now.tv_nsec,
                     (long)getpid(), (long)syscall(SYS_gettid),
                     handle, status, safe_module);
    if (n > 0) {
        size_t length = (size_t)n < sizeof(buffer) ? (size_t)n : sizeof(buffer) - 1;
        /* Diagnostics are best effort; never retry/block shutdown.
         * A short write leaves an incomplete line and fails parsing. */
        ssize_t written = write(STDERR_FILENO, buffer, length);
        if (written < 0 || (size_t)written != length) {
            in_trace = 0;
            return;
        }
    }
    in_trace = 0;
}

/* M23Z: snapshot executable PT_LOAD ranges once, before the first dlclose.
 * Output is raw ASLR-address evidence: it must remain in private local logs.
 * It does not assume that dlclose actually unmaps the named library.
 */
struct m23z_snapshot_state {
    unsigned int count;
    unsigned int truncated;
};

static int m23z_capture_exec(const struct dl_phdr_info *info, size_t size,
                             void *opaque) {
    (void)size;
    struct m23z_snapshot_state *state = opaque;
    const char *path = info->dlpi_name ? info->dlpi_name : "";
    const char *last = strrchr(path, '/');
    const char *name = last ? last + 1 : path;
    if (!name[0]) {
        name = "<main>";
    }
    for (size_t j = 0; j < info->dlpi_phnum; ++j) {
        const ElfW(Phdr) *ph = &info->dlpi_phdr[j];
        if (ph->p_type != PT_LOAD || !(ph->p_flags & PF_X) || !ph->p_memsz) {
            continue;
        }
        if (state->count >= 400U) {
            state->truncated = 1U;
            return 1;
        }
        uintptr_t base = (uintptr_t)info->dlpi_addr;
        if ((uintptr_t)ph->p_vaddr > UINTPTR_MAX - base) {
            state->truncated = 1U;
            return 1;
        }
        uintptr_t start = base + (uintptr_t)ph->p_vaddr;
        if ((uintptr_t)ph->p_memsz > UINTPTR_MAX - start) {
            state->truncated = 1U;
            return 1;
        }
        uintptr_t end = start + (uintptr_t)ph->p_memsz;
        char line[360];
        int len = snprintf(line, sizeof(line),
                           "M23Z_EXEC_MAP start=0x%lx end=0x%lx module=%.96s\n",
                           (unsigned long)start, (unsigned long)end, name);
        if (len <= 0 || (size_t)len >= sizeof(line) ||
            write(STDERR_FILENO, line, (size_t)len) != len) {
            state->truncated = 1U;
            return 1;
        }
        state->count++;
    }
    return 0;
}

static int m23z_snapshot_done = 0;
static void m23z_snapshot_before_first_close(void) {
    const char *guard = getenv("M23Z_MAP_ONLY");
    if (!enabled || in_trace || !guard || strcmp(guard, "1") != 0 ||
        !__sync_bool_compare_and_swap(&m23z_snapshot_done, 0, 1)) {
        return;
    }
    in_trace = 1;
    static const char start[] = "M23Z_MAP_BEGIN=YES\n";
    (void)write(STDERR_FILENO, start, sizeof(start) - 1);
    struct m23z_snapshot_state state = {0, 0};
    (void)dl_iterate_phdr(m23z_capture_exec, &state);
    char tail[120];
    int len = snprintf(tail, sizeof(tail),
                       "M23Z_MAP_END count=%u truncated=%u\n",
                       state.count, state.truncated);
    if (len > 0 && (size_t)len < sizeof(tail)) {
        (void)write(STDERR_FILENO, tail, (size_t)len);
    }
    in_trace = 0;
}

__attribute__((constructor))
static void initialize_trace(void) {
    const char *gate = getenv("M23R_TRACE_ONLY");
    if (gate == NULL || strcmp(gate, "1") != 0 || !is_component_container()) {
        return;
    }
    enabled = 1;
    next_dlopen = (void *(*)(const char *, int))dlsym(RTLD_NEXT, "dlopen");
    next_dlclose = (int (*)(void *))dlsym(RTLD_NEXT, "dlclose");
    event("CONSTRUCTOR", "component_container", NULL,
          (next_dlopen && next_dlclose) ? 0 : -1);
}

__attribute__((destructor))
static void finish_trace(void) {
    event("DESTRUCTOR", "component_container", NULL, 0);
}

void *dlopen(const char *filename, int flags) {
    if (!next_dlopen) {
        next_dlopen = (void *(*)(const char *, int))dlsym(RTLD_NEXT, "dlopen");
    }
    if (!next_dlopen) {
        return NULL;
    }
    void *handle = next_dlopen(filename, flags);
    event("DLOPEN_RETURN", filename, handle, handle ? 0 : -1);
    return handle;
}

int dlclose(void *handle) {
    if (!next_dlclose) {
        next_dlclose = (int (*)(void *))dlsym(RTLD_NEXT, "dlclose");
    }
    if (!next_dlclose) {
        return -1;
    }
    char path[448] = "<unknown>";
    /* glibc declares dlclose(handle) nonnull. Respect the contract;
     * don't compare a nonnull-annotated argument against NULL. */
    if (enabled && !in_trace) {
        struct link_map *mapping = NULL;
        if (dlinfo(handle, RTLD_DI_LINKMAP, &mapping) == 0 &&
            mapping && mapping->l_name && mapping->l_name[0]) {
            (void)snprintf(path, sizeof(path), "%.400s", mapping->l_name);
        }
    }
    event("DLCLOSE_BEGIN", path, handle, 0);
    m23z_snapshot_before_first_close();
    int status = next_dlclose(handle);
    event("DLCLOSE_RETURN", path, handle, status);
    return status;
}
