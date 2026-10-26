// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#ifndef GRAPH_DECODER_ATOMIC_FILE_H
#define GRAPH_DECODER_ATOMIC_FILE_H

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

namespace graph_file {

// Stage in the destination directory so rename replaces an old graph only after
// the writer and the flush finish. The temporary file is removed on failure.
template <typename Writer>
void AtomicWrite(const std::string& path, Writer writer) {
    if (path.empty()) throw std::invalid_argument("graph file path is empty");
    const std::string pattern = path + ".tmp.XXXXXX";
    std::vector<char> temporary(pattern.begin(), pattern.end());
    temporary.push_back('\0');
    int fd = ::mkstemp(temporary.data());
    if (fd < 0) throw std::system_error(errno, std::generic_category(), "cannot create graph temp file");
    std::FILE* stream = ::fdopen(fd, "w");
    if (stream == nullptr) {
        const int error = errno;
        ::close(fd);
        std::remove(temporary.data());
        throw std::system_error(error, std::generic_category(), "cannot open graph temp stream");
    }

    try {
        struct stat previous{};
        if (::stat(path.c_str(), &previous) == 0 && S_ISREG(previous.st_mode) &&
            ::fchmod(fd, previous.st_mode & 0777) != 0) {
            throw std::system_error(errno, std::generic_category(), "cannot preserve graph file mode");
        }
        writer(stream);
        if (std::ferror(stream)) throw std::system_error(EIO, std::generic_category(), "cannot write graph temp file");
        if (std::fflush(stream) != 0) throw std::system_error(errno, std::generic_category(), "cannot flush graph temp file");
        if (::fsync(fd) != 0) throw std::system_error(errno, std::generic_category(), "cannot sync graph temp file");
        const int close_status = std::fclose(stream);
        stream = nullptr;
        if (close_status != 0) throw std::system_error(errno, std::generic_category(), "cannot close graph temp file");
        if (std::rename(temporary.data(), path.c_str()) != 0)
            throw std::system_error(errno, std::generic_category(), "cannot replace graph file");
    } catch (...) {
        if (stream != nullptr) std::fclose(stream);
        std::remove(temporary.data());
        throw;
    }
}

}  // namespace graph_file

#endif
