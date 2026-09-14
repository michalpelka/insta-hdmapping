// A minimal child-process wrapper: spawns argv[0] with argv (no shell involved, so
// paths and arguments never need escaping), and gives the caller a pipe to read its
// stdout plus the ability to fetch its stderr for error reporting. Every use in this
// project passes input via a file path argument (never stdin), so this only ever
// needs to stream output out of the child -- the same shape as Python's
// subprocess.Popen(argv, stdout=PIPE, stderr=tempfile) in the original tool.
#pragma once

#include <string>
#include <vector>

#include <sys/types.h>

namespace insta360 {

class Subprocess {
public:
    // Spawns argv[0] with the given arguments. stdout is captured via a pipe this
    // object owns the read end of; stderr goes to a private temp file so the child
    // never blocks on a full pipe while we are busy reading stdout.
    explicit Subprocess(const std::vector<std::string>& argv);
    ~Subprocess();

    Subprocess(const Subprocess&) = delete;
    Subprocess& operator=(const Subprocess&) = delete;

    // Reads up to `size` bytes from the child's stdout into `buf`; returns the number
    // of bytes actually read, 0 at EOF.
    size_t read_stdout(void* buf, size_t size);

    // Marks that the caller drained stdout to natural EOF (as opposed to stopping
    // early); only then is a non-zero exit status treated as an error by close().
    void mark_exhausted() { exhausted_ = true; }

    // Kills the process if it is still running and waits for it; call this instead of
    // letting the destructor do it when you want to check the outcome via
    // last_error().
    void close();

    // Empty if the process is still running, exited 0, or was killed deliberately
    // (mark_exhausted() not called); otherwise a message including the stderr tail.
    std::string last_error(const std::string& context) const;

private:
    std::vector<std::string> argv_;
    pid_t pid_ = -1;
    int stdout_fd_ = -1;
    std::string stderr_path_;
    bool exhausted_ = false;
    bool closed_ = false;
    bool killed_ = false;
    int exit_status_ = 0;

    std::string stderr_tail(size_t limit = 2000) const;
};

}  // namespace insta360
