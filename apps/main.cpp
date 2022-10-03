#include <boost/program_options.hpp>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>

#include "fuser/fuser.hpp"

bool parse_arguments(int argc, char const* argv[], std::string& config_path,
                     std::string& input_path, std::string& output_folder) {
  namespace po = boost::program_options;
  try {
    po::options_description desc("Fuser example app", 1024, 512);
    desc.add_options()("help,h", "This message")(
        "config,c", po::value<std::string>(&config_path)->required(),
        "Path to the file with config")(
        "input,i", po::value<std::string>(&input_path)->required(),
        "Path to the file with input data")(
        "output,o", po::value<std::string>(&output_folder)->required(),
        "Path to the output directory");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);

    if (vm.count("help")) {
      std::cout << desc << "\n";
      return false;
    }

    po::notify(vm);
  } catch (std::exception& e) {
    std::cerr << "Error: " << e.what() << "\n";
    return false;
  } catch (...) {
    std::cerr << "Unknown error!"
              << "\n";
    return false;
  }

  return true;
}

int main(int argc, char const* argv[]) {
  std::string config_path;
  std::string input_path;
  std::string output_folder;
  auto result =
      parse_arguments(argc, argv, config_path, input_path, output_folder);

  if (!result) {
    return -1;
  }

  auto json_config = nlohmann::json::parse(std::ifstream(config_path));
  fuser::FuserConfig fuser_config(json_config);

  auto data = nlohmann::json::parse(std::ifstream(input_path));
  auto object = data.contains("object") ? fuser::Object(data.at("object"))
                                        : fuser::Object();

  fuser::Fuser fuser(fuser_config);
  fuser.SetObject(object);
  std::cout << fuser.GetObject();

  for (const auto& json_measurement : data.at("measurements")) {
    fuser::Measurement measurement(json_measurement);
    std::cout << measurement;

    fuser.ProcessMeasurement(measurement);

    std::cout << fuser.GetObject();
  }

  return 0;
}
