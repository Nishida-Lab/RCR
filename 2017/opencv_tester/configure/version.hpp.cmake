#ifndef INCLUDED_DEBUGGER_VERSION_HPP_
#define INCLUDED_DEBUGGER_VERSION_HPP_


#include <debugger/string/static_concatenate.hpp>


static constexpr auto project_major_version {scat("${PROJECT_VERSION_MAJOR}")};
static constexpr auto project_minor_version {scat("${PROJECT_VERSION_MINOR}")};
static constexpr auto project_patch_version {scat("${PROJECT_VERSION_PATCH}")};

static constexpr auto project_version {scat("${PROJECT_VERSION}")};


#endif
