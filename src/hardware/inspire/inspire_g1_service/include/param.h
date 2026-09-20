#ifndef PARAM_H
#define PARAM_H

#include <stdint.h>
#include <iostream>
#include <chrono>
#include <boost/program_options.hpp>

namespace param
{

namespace po = boost::program_options;

inline std::string serial_port;
inline std::string network; 
inline std::string ns; 
inline float threhold;

po::variables_map helper(int argc, char** argv)
{
  po::options_description desc("Unitree H1 Inspire Hand Serial to DDS");
  desc.add_options()
    ("help,h", "produce help message")
    ("serial,s", po::value<std::string>(&serial_port)->default_value("auto"), "serial port (default: auto-detect)")
    ("network", po::value<std::string>(&network)->default_value(""), "DDS network interface")
    ("namespace", po::value<std::string>(&ns)->default_value("inspire"), "DDS topic namespace")
    ;

  po::variables_map vm;
  po::store(po::parse_command_line(argc, argv, desc), vm);
  po::notify(vm);

  if (vm.count("help"))
  {
    std::cout << desc << std::endl;
    exit(0);
  }

  if(ns.empty())
  {
    std::cerr << "Namespace cannot be empty" << std::endl;
    exit(1);
  }

  return vm;
}

}

#endif // PARAM_H
