#ifndef INCLUDED_UTILIB_UNIX_UNIQUE_FD_HPP_
#define INCLUDED_UTILIB_UNIX_UNIQUE_FD_HPP_


#include <utility>   // std::swap, std::exchange
#include <unistd.h>  // ::close


namespace utilib {


class unique_fd
{
  int fd_ = -1;

public:
  constexpr unique_fd() noexcept = default;

  explicit unique_fd(int fd) noexcept
    : fd_ {fd}
  {}

  explicit unique_fd(unique_fd&& u) noexcept
    : fd_ {u.release()}
  {}

  ~unique_fd() noexcept { ::close(fd_); }

  unique_fd(const unique_fd&) = delete;

  unique_fd& operator=(const unique_fd&) = delete;

  operator int() const noexcept { return fd_; }
  operator bool() const noexcept { return fd_ != -1; }

  int release() noexcept { return std::exchange(fd_, -1); }

  void swap(unique_fd& ufd) noexcept { std::swap(fd_, ufd.fd_); }
  friend void swap(unique_fd& lhs, unique_fd& rhs) noexcept { lhs.swap(rhs); }

  void reset(int fd = -1) noexcept { unique_fd(fd).swap(*this); }
};


} // namespace utilib

#endif
