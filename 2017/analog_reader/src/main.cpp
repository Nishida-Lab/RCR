#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <utility>
#include <vector>


int main(int argc, char** argv)
{
  std::ifstream file {"/home/yamasa/works/tmp/dummy_output.txt"};
  std::vector<std::vector<unsigned long>> text_buffer {};

  if (file.is_open())
  {
    for (std::string word_buffer {}; std::getline(file, word_buffer); )
    {
      std::vector<unsigned long> line_buffer {};

      for (std::stringstream ss {word_buffer}; std::getline(ss, word_buffer, ' '); )
      {
        word_buffer.erase(word_buffer.begin(), word_buffer.begin() + 2);
        line_buffer.push_back(std::stoi(word_buffer));
      }

      text_buffer.emplace_back(std::move(line_buffer));
    }

    file.close();
  }

  else {
    std::cerr << "[error] failed to open file\n";
    std::exit(EXIT_FAILURE);
  }


  for (const auto& line : text_buffer)
  {
    for (const auto& word : line)
    {
      std::cout << word << (&word != &line.back() ? ' ' : '\n');
    }
  }

  return 0;
}


