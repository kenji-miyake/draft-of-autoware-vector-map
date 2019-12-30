#include <autoware_vector_map/io/gpkg_interface.h>

#include <fstream>
#include <random>
#include <string>

#include <autoware_vector_map/future/filesystem.h>

namespace autoware_vector_map {
namespace io {

GpkgInterface::GpkgInterface(const char* gpkg_path) {
  GDALAllRegister();

  if (!std::filesystem::exists(gpkg_path)) {
    const auto msg = fmt::format("no such file: {}", gpkg_path);
    throw std::runtime_error(msg);
  }

  dataset_.reset(
      static_cast<GDALDataset*>(GDALOpenEx(gpkg_path, GDAL_OF_VECTOR, nullptr, nullptr, nullptr)));
  if (!dataset_) {
    throw std::runtime_error("failed to initialize dataset");
  }
}

GpkgInterface::GpkgInterface(const std::vector<uint8_t>& bin_data) {
  GDALAllRegister();

  constexpr const char* vsi_file_name = "/vsimem/memory.gpkg";

  const auto ptr = VSIFileFromMemBuffer(vsi_file_name, const_cast<GByte*>(bin_data.data()),
                                        bin_data.size(), false);
  if (!ptr) {
    throw std::runtime_error("failed to create VSI file");
  }

  dataset_.reset(static_cast<GDALDataset*>(
      GDALOpenEx(vsi_file_name, GDAL_OF_VECTOR | GDAL_OF_READONLY, nullptr, nullptr, nullptr)));
  if (!dataset_) {
    throw std::runtime_error("failed to initialize dataset");
  }

  VSIFCloseL(ptr);
  VSIUnlink(vsi_file_name);
}

void GpkgInterface::toFile(const char* gpkg_path) {
  if (!dataset_) {
    throw std::runtime_error("dataset_ is empty");
  }

  GDALDriver* gpkg_driver = GetGDALDriverManager()->GetDriverByName("GPKG");
  const auto dataset = std::shared_ptr<GDALDataset>(
      gpkg_driver->CreateCopy(gpkg_path, dataset_.get(), false, nullptr, nullptr, nullptr));
}

std::vector<uint8_t> GpkgInterface::toBinary() {
  // Generate random string
  std::string random_str;
  {
    std::random_device rd{};
    std::mt19937 generator{rd()};
    std::uniform_int_distribution<int> distribution{'a', 'z'};

    for (size_t i = 0; i < 10; ++i) {
      random_str.push_back(distribution(generator));
    }
  }

  // Create temporary copy
  const auto temp_file_name = fmt::format("temp_{}.gpkg", random_str);
  const auto copied_gpkg_path = std::filesystem::temp_directory_path() / temp_file_name;
  toFile(copied_gpkg_path.c_str());

  // Read file and copy to binary data
  std::vector<uint8_t> bin_data;
  {
    std::ifstream ifs(copied_gpkg_path.c_str(), std::ios_base::binary);
    if (!ifs) {
      throw std::runtime_error("failed to open file");
    }

    // Seek cursor and get file size
    ifs.seekg(0, std::ios_base::end);
    const size_t file_size = static_cast<size_t>(ifs.tellg());
    ifs.seekg(0, std::ios_base::beg);

    bin_data.resize(file_size);
    ifs.read(reinterpret_cast<char*>(bin_data.data()), file_size);
  }

  // Remove temporary file
  std::filesystem::remove(copied_gpkg_path);

  return bin_data;
}

}  // namespace io
}  // namespace autoware_vector_map
