#include "insta360/subprocess.hpp"

#include <fcntl.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <stdexcept>

extern char** environ;

namespace insta360 {

namespace {

std::string join(const std::vector<std::string>& argv) {
    std::string out;
    for (size_t i = 0; i < argv.size(); ++i) {
        if (i) out += ' ';
        out += argv[i];
    }
    return out;
}

std::string make_temp_path() {
    const char* tmpdir = std::getenv("TMPDIR");
    std::string tmpl = (tmpdir ? std::string(tmpdir) : std::string("/tmp")) +
                        "/insta360_to_images_stderr_XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back('\0');
    int fd = mkstemp(buf.data());
    if (fd < 0) throw std::runtime_error("could not create a temp file for subprocess stderr");
    ::close(fd);
    return std::string(buf.data());
}

}  // namespace

Subprocess::Subprocess(const std::vector<std::string>& argv) : argv_(argv) {
    if (argv_.empty()) throw std::runtime_error("Subprocess: empty argument list");
    stderr_path_ = make_temp_path();

    int stdout_pipe[2];
    if (pipe(stdout_pipe) != 0) {
        throw std::runtime_error("pipe() failed for subprocess stdout");
    }

    std::vector<char*> exec_argv;
    exec_argv.reserve(argv_.size() + 1);
    for (auto& arg : argv_) exec_argv.push_back(const_cast<char*>(arg.c_str()));
    exec_argv.push_back(nullptr);

    posix_spawn_file_actions_t actions;
    posix_spawn_file_actions_init(&actions);
    posix_spawn_file_actions_addclose(&actions, stdout_pipe[0]);
    posix_spawn_file_actions_adddup2(&actions, stdout_pipe[1], STDOUT_FILENO);
    posix_spawn_file_actions_addclose(&actions, stdout_pipe[1]);
    posix_spawn_file_actions_addopen(&actions, STDERR_FILENO, stderr_path_.c_str(),
                                      O_WRONLY | O_TRUNC, 0600);

    pid_t pid = 0;
    int rc = posix_spawnp(&pid, exec_argv[0], &actions, nullptr, exec_argv.data(), environ);
    posix_spawn_file_actions_destroy(&actions);
    ::close(stdout_pipe[1]);
    if (rc != 0) {
        ::close(stdout_pipe[0]);
        throw std::runtime_error("failed to launch '" + join(argv_) + "': " + std::strerror(rc));
    }
    pid_ = pid;
    stdout_fd_ = stdout_pipe[0];
}

Subprocess::~Subprocess() {
    close();
    ::unlink(stderr_path_.c_str());
}

size_t Subprocess::read_stdout(void* buf, size_t size) {
    if (stdout_fd_ < 0) return 0;
    ssize_t n = ::read(stdout_fd_, buf, size);
    if (n < 0) {
        throw std::runtime_error("read from subprocess stdout failed: " +
                                  std::string(std::strerror(errno)));
    }
    return static_cast<size_t>(n);
}

void Subprocess::close() {
    if (closed_) return;
    closed_ = true;
    if (stdout_fd_ >= 0) {
        ::close(stdout_fd_);
        stdout_fd_ = -1;
    }
    if (pid_ > 0) {
        if (!exhausted_) {
            ::kill(pid_, SIGKILL);
            killed_ = true;
        }
        int status = 0;
        while (waitpid(pid_, &status, 0) < 0 && errno == EINTR) {
        }
        exit_status_ = status;
    }
}

std::string Subprocess::stderr_tail(size_t limit) const {
    std::ifstream file(stderr_path_, std::ios::binary);
    if (!file) return "";
    file.seekg(0, std::ios::end);
    std::streamoff size = file.tellg();
    std::streamoff start =
        size > static_cast<std::streamoff>(limit) ? size - static_cast<std::streamoff>(limit) : 0;
    file.seekg(start);
    std::ostringstream out;
    out << file.rdbuf();
    return out.str();
}

std::string Subprocess::last_error(const std::string& context) const {
    if (!closed_ || killed_ || !exhausted_) return "";
    if (WIFEXITED(exit_status_) && WEXITSTATUS(exit_status_) == 0) return "";
    std::string tail = stderr_tail();
    std::string message = context;
    if (WIFEXITED(exit_status_)) {
        message += " exited with status " + std::to_string(WEXITSTATUS(exit_status_));
    } else if (WIFSIGNALED(exit_status_)) {
        message += " was killed by signal " + std::to_string(WTERMSIG(exit_status_));
    } else {
        message += " ended abnormally";
    }
    if (!tail.empty()) message += ":\n" + tail;
    return message;
}

}  // namespace insta360
