// Copyright (c) 2026 Fan Yang and contributors
// SPDX-License-Identifier: MIT

#ifndef VISIBILITY_GRAPH_MSG_GRAPH_FILE_WRITER_H
#define VISIBILITY_GRAPH_MSG_GRAPH_FILE_WRITER_H

#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <limits>
#include <stdexcept>
#include <string>
#include <system_error>
#include <vector>

#include <fcntl.h>
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
    const int reserved_fd = ::mkstemp(temporary.data());
    if (reserved_fd < 0) throw std::system_error(errno, std::generic_category(), "cannot reserve graph temp path");
    if (::close(reserved_fd) != 0 || ::unlink(temporary.data()) != 0) {
        const int error = errno;
        std::remove(temporary.data());
        throw std::system_error(error, std::generic_category(), "cannot prepare graph temp path");
    }
    // O_EXCL protects the gap after unlink; mode 0666 honors the process umask,
    // matching the previous ofstream behavior for newly created graph files.
    int fd = ::open(temporary.data(), O_WRONLY | O_CREAT | O_EXCL | O_CLOEXEC, 0666);
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

// Both the decoder and boundary extractor store the same graph fields.
// Float max_digits10 preserves the stored coordinates across a text round trip.
template <typename Nodes>
void WriteGraphFile(const std::string& path, const Nodes& nodes) {
    AtomicWrite(path, [&](std::FILE* file) {
        constexpr int digits = std::numeric_limits<float>::max_digits10;
        const auto output_point = [&](const auto& point) {
            std::fprintf(file, "%.*g %.*g %.*g ", digits, static_cast<double>(point.x),
                         digits, static_cast<double>(point.y), digits, static_cast<double>(point.z));
        };
        const auto output_ids = [&](const auto& ids) {
            for (const auto id : ids) std::fprintf(file, "%zu ", id);
        };
        for (const auto& node : nodes) {
            std::fprintf(file, "%zu %d ", node->id, static_cast<int>(node->free_direct));
            output_point(node->position);
            output_point(node->surf_dirs.first);
            output_point(node->surf_dirs.second);
            std::fprintf(file, "%d %d %d %d ", static_cast<int>(node->is_covered),
                         static_cast<int>(node->is_frontier), static_cast<int>(node->is_navpoint),
                         static_cast<int>(node->is_boundary));
            output_ids(node->connect_idxs);
            std::fputs("| ", file);
            output_ids(node->poly_idxs);
            std::fputs("| ", file);
            output_ids(node->contour_idxs);
            std::fputs("| ", file);
            output_ids(node->traj_idxs);
            std::fputc('\n', file);
        }
    });
}

}  // namespace graph_file

#endif
