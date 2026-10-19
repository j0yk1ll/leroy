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
        (void)write(STDERR_FILENO, buffer, length);
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
    if (enabled && !in_trace && handle != NULL) {
        struct link_map *mapping = NULL;
        if (dlinfo(handle, RTLD_DI_LINKMAP, &mapping) == 0 &&
            mapping && mapping->l_name && mapping->l_name[0]) {
            (void)snprintf(path, sizeof(path), "%.400s", mapping->l_name);
        }
    }
    event("DLCLOSE_BEGIN", path, handle, 0);
    int status = next_dlclose(handle);
    event("DLCLOSE_RETURN", path, handle, status);
    return status;
}
