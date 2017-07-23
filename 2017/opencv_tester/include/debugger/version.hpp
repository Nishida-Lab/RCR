#ifndef INCLUDED_DEBUGGER_VERSION_HPP_
#define INCLUDED_DEBUGGER_VERSION_HPP_


#include <debugger/string/static_concatenate.hpp>


static constexpr auto project_major_version {scat("0")};
static constexpr auto project_minor_version {scat("0")};
static constexpr auto project_patch_version {scat("0")};

static constexpr auto project_version {scat("0.0.0")};


#endif
