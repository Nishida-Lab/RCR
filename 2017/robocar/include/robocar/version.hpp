#ifndef INCLUDED_ROBOCAR_VERSION_HPP_
#define INCLUDED_ROBOCAR_VERSION_HPP_


#include <string>

#include <boost/version.hpp>


const std::string project_major_version {"0"};
const std::string project_minor_version {"0"};
const std::string project_patch_version {"0"};

const std::string project_version {"0.0.0"};


const std::string boost_major_version {std::to_string(BOOST_VERSION / 100000)};
const std::string boost_minor_version {std::to_string(BOOST_VERSION / 100 % 1000)};
const std::string boost_patch_version {std::to_string(BOOST_VERSION % 100)};

const std::string boost_version {boost_major_version + "." + boost_minor_version + "." + boost_patch_version};


const std::string cmake_build_type {"Debug"};


#endif
