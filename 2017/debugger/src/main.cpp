#include <iostream>
#include <regex>
#include <string>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <debugger/version.hpp>


int main(int argc, char** argv)
{
  const std::vector<std::string> argv_ {argv, argv + argc};

  if (1 < argv_.size())
  {
    for (auto iter {std::begin(argv_)}; iter != std::end(argv_); ++iter)
    {
      for (const auto& s : decltype(argv_) {"^-v$", "^--version$"})
      {
        if (std::regex_match(*iter, std::regex {s}))
        {
          return !(std::cout << "version " << project_version.data() << " alpha\n").good();
        }
      }
    }
  }

  return 0;
}
