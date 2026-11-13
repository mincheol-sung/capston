// Read the installed Cartographer format through its own deserializer.
#include <fstream>
#include <iostream>
#include <stdexcept>

#include "cartographer/io/proto_stream_deserializer.h"
#include "google/protobuf/util/json_util.h"

int main(int argc, char** argv) {
  if (argc != 3) {
    std::cerr << "Usage: export_pose_graph input.pbstream output.json\n";
    return 2;
  }
  auto graph = cartographer::io::DeserializePoseGraphFromFile(argv[1]);
  google::protobuf::util::JsonPrintOptions options;
  options.always_print_primitive_fields = true;
  options.preserve_proto_field_names = true;
  std::string json;
  const auto status = google::protobuf::util::MessageToJsonString(graph, &json, options);
  if (!status.ok()) throw std::runtime_error(status.ToString());
  std::ofstream out(argv[2]);
  out << json << '\n';
  if (!out) throw std::runtime_error("Cannot write graph JSON");
  std::cout << graph.trajectory_size() << " trajectories, "
            << graph.constraint_size() << " constraints exported\n";
}
